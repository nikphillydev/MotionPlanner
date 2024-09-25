#include "at_motion_planner/navigator.hpp"

Navigator::Navigator()
    : debug_{false},
      startpoint_tolerance_{0.10},
      goal_tolerance_{0.20},            // 10 cm
      angular_goal_tolerance_{0.1745},  // 10 degrees
      max_speed_{0.30},
      max_angular_speed_{1.11},
      movement_state_{MovementState::START},
      path_length_{},
      targets_{},
      path_{},
      current_path_itr_{path_.begin()},
      next_path_itr_{path_.end()},
      itr_ready_{false} {}

/* ------ Motion planner interface ------ */

bool Navigator::create_path(const std::vector<geometry_msgs::msg::Pose>& targets,
                            int poses_per_meter, int orient_mode,
                            const geometry_msgs::msg::Quaternion& orientation, bool level_lock,
                            rclcpp::Logger logger) {
  if (targets.size() < 2) {
    RCLCPP_ERROR(logger, "Less than two targets provided for path creation");
    return false;
  }

  RCLCPP_INFO(logger, "Starting spline interpolation to create path poses between targets ...");

  // Reset path and targets
  reset_path();

  // Set targets on path
  targets_ = targets;

  // Extract x,y,z coordinates from target poses
  std::vector<double> X{};
  std::vector<double> Y{};
  std::vector<double> Z{};
  for (size_t i = 0; i < targets_.size(); i++) {
    X.push_back(targets_.at(i).position.x);
    Y.push_back(targets_.at(i).position.y);
    Z.push_back(targets_.at(i).position.z);
  }

  // Normalize every target pose orientation
  for (auto& pose : targets_) {
    Eigen::Quaterniond q{pose.orientation.w, pose.orientation.x, pose.orientation.y,
                         pose.orientation.z};
    q.normalize();
    pose.set__orientation(nav::create_quaternion(q.x(), q.y(), q.z(), q.w()));
  }

  // Create parametric forms of the path for x,y,z coordinates.
  // Time proportional to distance, therefore use distance as the
  // free parameter ranging [0, total length]
  std::vector<double> D{0.0};
  for (size_t i = 1; i < targets_.size(); i++) {
    double x_term = (X.at(i) - X.at(i - 1)) * (X.at(i) - X.at(i - 1));
    double y_term = (Y.at(i) - Y.at(i - 1)) * (Y.at(i) - Y.at(i - 1));
    double z_term = (Z.at(i) - Z.at(i - 1)) * (Z.at(i) - Z.at(i - 1));
    double running_dist =
        std::max(D.at(i - 1) + sqrt(x_term + y_term + z_term), D.at(i - 1) + 0.05);
    D.push_back(running_dist);
  }
  tk::spline sx{D, X, tk::spline::cspline, false};
  tk::spline sy{D, Y, tk::spline::cspline, false};
  tk::spline sz{D, Z, tk::spline::cspline, false};

  // Construct poses along curve, add to path
  path_length_ = D.back();
  double num_poses = path_length_ * (double)poses_per_meter;

  size_t target_index = 0;
  std::vector<size_t> delta_poses(targets_.size(),
                                  1);  // number of poses added in between each target

  for (size_t pose_count = 0; pose_count <= num_poses; pose_count++) {
    double running_dist = (double)pose_count / (double)poses_per_meter;
    if (std::abs(running_dist - D.at(target_index)) <= 0.001) {
      // Running distance is equal to target distance, add target
      path_.push_back(targets_.at(target_index));
      target_index++;
      continue;
    } else if ((running_dist - D.at(target_index)) > 0.001) {
      // Running distance is greater than target distance, add target then interpolated pose
      path_.push_back(targets_.at(target_index));
      target_index++;
    }
    // Add interpolated pose
    double x_pos = sx(running_dist);
    double y_pos = sy(running_dist);
    double z_pos = sz(running_dist);
    path_.push_back(nav::create_pose(x_pos, y_pos, z_pos, 0.0, 0.0, 0.0, 1.0));
    delta_poses.at(target_index)++;
  }

  for (size_t i = target_index; i < targets_.size(); i++) {
    // Add remaining targets to path
    path_.push_back(targets_.at(i));
  }

  // Compute the (more accurate) path length
  path_length_ = 0.0;
  for (std::vector<geometry_msgs::msg::Pose>::const_iterator curr{path_.begin()};
       curr != path_.end() - 1; curr++) {
    std::vector<geometry_msgs::msg::Pose>::const_iterator next{curr + 1};
    float x_term = ((next->position.x - curr->position.x) * (next->position.x - curr->position.x));
    float y_term = ((next->position.y - curr->position.y) * (next->position.y - curr->position.y));
    float z_term = ((next->position.z - curr->position.z) * (next->position.z - curr->position.z));
    path_length_ += sqrt(x_term + y_term + z_term);
  }

  // Restart movement. NOTE: Must be TRAVERSE for waypoint mode
  movement_state_ = MovementState::TRAVERSE;

  // Set path iterators
  current_path_itr_ = path_.begin();
  next_path_itr_ = path_.begin() + 1;
  itr_ready_ = true;

  RCLCPP_INFO(logger, "Pose spline interpolation success");
  RCLCPP_INFO(logger, "Initializing pose orientations for orient_mode = %d ...", orient_mode);

  // Set orientations of each pose according to requested orientation mode
  switch (orient_mode) {
    case 0: {
      RCLCPP_INFO(logger, "\t-Orient towards direction of travel, offset applied");
      // Each pose on the path points to the next, then add relative offset
      for (std::vector<geometry_msgs::msg::Pose>::iterator curr{path_.begin()}; curr != path_.end();
           curr++) {
        std::vector<geometry_msgs::msg::Pose>::const_iterator next{curr + 1};

        // Compute forward facing X axis vector
        Eigen::Vector3d X;
        if (next == path_.end()) {
          // Extrapolate linearly to find point after endpoint
          double over_length = path_length_ + 1;
          X << sx(over_length) - curr->position.x, sy(over_length) - curr->position.y,
              sz(over_length) - curr->position.z;
        } else {
          X << next->position.x - curr->position.x, next->position.y - curr->position.y,
              next->position.z - curr->position.z;
        }
        X.normalize();

        // Compute right facing Y axis vector: project X onto xy plane, rotate
        // 90 degrees clockwise about z-axis (no roll wanted)
        Eigen::Vector3d Y;
        Y << X(0), X(1), 0.0;
        Y.normalize();
        Eigen::Matrix3d rm;
        rm << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        Y = rm * Y;

        // Compute upward facing Z axis vector: cross product X and Y
        Eigen::Vector3d Z{X.cross(Y)};
        Z.normalize();

        // { X, Y, Z } is a orthonormal basis describing the full rotation of the pose.
        // Construct a rotation matrix
        Eigen::Matrix3d qm;
        qm.col(0) = X;
        qm.col(1) = Y;
        qm.col(2) = Z;

        // Convert rotation matrix into quaternion and apply rotation offset
        Eigen::Quaterniond q{qm};
        Eigen::Quaterniond offset{orientation.w, orientation.x, orientation.y, orientation.z};
        offset.normalize();
        q *= offset;

        if (level_lock) {
          // Set roll and pitch to 0
          std::array<float, 3> euler{nav::quaternion_to_euler(q.x(), q.y(), q.z(), q.w())};
          q = nav::euler_to_quaternion(0.0, 0.0, euler.at(2));
        }

        // Set pose orientation
        curr->set__orientation(nav::create_quaternion(q.x(), q.y(), q.z(), q.w()));
      }
      break;
    }

    case 1: {
      RCLCPP_INFO(logger, "\t-Constant orientation");
      // Each pose has a constant orientation
      Eigen::Quaterniond q{orientation.w, orientation.x, orientation.y, orientation.z};
      q.normalize();

      if (level_lock) {
        // Set roll and pitch to 0
        std::array<float, 3> euler{nav::quaternion_to_euler(q.x(), q.y(), q.z(), q.w())};
        q = nav::euler_to_quaternion(0.0, 0.0, euler.at(2));
      }

      // Set pose orientations
      for (std::vector<geometry_msgs::msg::Pose>::iterator curr{path_.begin()}; curr != path_.end();
           curr++) {
        curr->set__orientation(nav::create_quaternion(q.x(), q.y(), q.z(), q.w()));
      }
      break;
    }

    case 2: {
      RCLCPP_INFO(logger, "\t-Interpolated orientation between targets");
      // Each pose interpolates its orientation between the previous and next target orientation
      std::vector<geometry_msgs::msg::Pose>::iterator curr{path_.begin()};
      size_t target_index = 0;

      while (curr != path_.end() - 1) {
        Eigen::Quaterniond q_init{
            targets_.at(target_index).orientation.w, targets_.at(target_index).orientation.x,
            targets_.at(target_index).orientation.y, targets_.at(target_index).orientation.z};
        Eigen::Quaterniond q_final{targets_.at(target_index + 1).orientation.w,
                                   targets_.at(target_index + 1).orientation.x,
                                   targets_.at(target_index + 1).orientation.y,
                                   targets_.at(target_index + 1).orientation.z};

        if (level_lock) {
          // Set roll and pitch for both initial and final orientations to 0
          std::array<float, 3> euler_init{
              nav::quaternion_to_euler(q_init.x(), q_init.y(), q_init.z(), q_init.w())};
          q_init = nav::euler_to_quaternion(0.0, 0.0, euler_init.at(2));

          std::array<float, 3> euler_final{
              nav::quaternion_to_euler(q_final.x(), q_final.y(), q_final.z(), q_final.w())};
          q_final = nav::euler_to_quaternion(0.0, 0.0, euler_final.at(2));
        }

        // Get number of poses between target orientations
        size_t num_poses = delta_poses.at(target_index + 1);

        // Compute scalar t [0..1] and interpolate quaternion pose
        for (size_t pose_count = 0; pose_count <= num_poses; pose_count++) {
          double t = (double)pose_count / (double)num_poses;
          Eigen::Quaterniond q{q_init.slerp(t, q_final)};
          curr->set__orientation(nav::create_quaternion(q.x(), q.y(), q.z(), q.w()));
          if (pose_count == num_poses) {
            // t = 1: at final orientation. Do not increment iterator so proper
            // num_poses can be calculated the next iteration
            continue;
          }
          curr++;
        }
        target_index++;
      }
      break;
    }

    default:
      // Default orientation of (w=0.0, x=0.0, y=0.0, z=0.0)
      RCLCPP_WARN(logger, "Invalid orient mode, defaulting to (w=1.0, x=0.0, y=0.0, z=0.0)");
      break;
  }

  RCLCPP_INFO(logger, "Path orientations initialized for all poses");

  return true;
}

void Navigator::reset_path() {
  itr_ready_ = false;
  path_length_ = 0;
  targets_ = std::vector<geometry_msgs::msg::Pose>{};
  path_ = std::vector<geometry_msgs::msg::Pose>{};
  current_path_itr_ = path_.begin();
  next_path_itr_ = path_.end();
}

bool Navigator::is_path_empty() const { return path_.empty(); };

std::array<float, 6> Navigator::seek_velocity(
    const geometry_msgs::msg::TransformStamped& world2robot_t_, rclcpp::Logger logger) {
  // Path to follow
  assert(!is_path_empty());

  // Get velocities to follow path
  std::array<float, 3> linear_vel = seek_linear(world2robot_t_, logger);
  std::array<float, 3> angular_vel = seek_angular(world2robot_t_, logger);

  // Return velocity target
  std::array<float, 6> velocity{linear_vel.at(0),  linear_vel.at(1),  linear_vel.at(2),
                                angular_vel.at(0), angular_vel.at(1), angular_vel.at(2)};
  return velocity;
}

bool Navigator::reached_goal(const geometry_msgs::msg::TransformStamped& world2robot_t_,
                             rclcpp::Logger logger) const {
  assert(!is_path_empty());

  // Check position
  bool within_position =
      (movement_state_ == MovementState::END) && (get_distance_remaining() <= goal_tolerance_);

  // Check orientation
  geometry_msgs::msg::Pose last_rel_target;
  geometry_msgs::msg::Pose last_target{path_.back()};
  tf2::doTransform(last_target, last_rel_target, world2robot_t_);
  Eigen::Quaternionf q{last_rel_target.orientation.w, last_rel_target.orientation.x,
                       last_rel_target.orientation.y, last_rel_target.orientation.z};
  Eigen::AngleAxisf aa(q);
  bool within_rotation = aa.angle() <= angular_goal_tolerance_;

  return within_position && within_rotation;
}

/* ------ Getters ------ */

float Navigator::get_distance_remaining() const {
  float distance_remaining = 0;

  // Distance along path from current position to the end
  for (std::vector<geometry_msgs::msg::Pose>::const_iterator curr{path_.end() - 1};
       curr > current_path_itr_; curr--) {
    std::vector<geometry_msgs::msg::Pose>::const_iterator next{curr - 1};
    float x_term = ((next->position.x - curr->position.x) * (next->position.x - curr->position.x));
    float y_term = ((next->position.y - curr->position.y) * (next->position.y - curr->position.y));
    float z_term = ((next->position.z - curr->position.z) * (next->position.z - curr->position.z));
    distance_remaining += sqrt(x_term + y_term + z_term);
  }

  return distance_remaining;
}

float Navigator::get_distance_travelled() const { return path_length_ - get_distance_remaining(); }

nav_msgs::msg::Path Navigator::get_path() const {
  nav_msgs::msg::Path nav_path{};
  nav_path.header.set__frame_id("world");
  for (const auto& pose : path_) {
    geometry_msgs::msg::PoseStamped ps{};
    ps.header.set__frame_id("world");
    ps.set__pose(pose);
    nav_path.poses.push_back(ps);
  }
  return nav_path;
}

geometry_msgs::msg::PoseArray Navigator::get_path_poses() const {
  geometry_msgs::msg::PoseArray pose_array{};
  pose_array.header.set__frame_id("world");
  for (const auto& pose : path_) {
    pose_array.poses.push_back(pose);
  }
  return pose_array;
}

geometry_msgs::msg::PoseArray Navigator::get_path_targets() const {
  geometry_msgs::msg::PoseArray pose_array{};
  pose_array.header.set__frame_id("world");
  for (const auto& pose : targets_) {
    pose_array.poses.push_back(pose);
  }
  if (itr_ready_) {
    pose_array.poses.push_back(*current_path_itr_);
  }
  return pose_array;
}

geometry_msgs::msg::Pose Navigator::get_current_pose() const {
  assert(!is_path_empty());

  return *current_path_itr_;
}

/* ------ Internal velocity calculation ------ */

std::array<float, 3> Navigator::seek_linear(
    const geometry_msgs::msg::TransformStamped& world2robot_t_, rclcpp::Logger logger) {
  // Path to follow
  assert(!is_path_empty());

  // Outgoing target velocity to follow the path
  std::array<float, 3> velocity{0, 0, 0};

  // Get current (closest) target pose on the path relative to robot
  geometry_msgs::msg::Pose curr_rel_target;
  geometry_msgs::msg::Pose curr_target{*current_path_itr_};
  tf2::doTransform(curr_target, curr_rel_target, world2robot_t_);

  // Relative [x, y, z] vector to current target pose
  Eigen::Vector3f curr_vec_ei{curr_rel_target.position.x, curr_rel_target.position.y,
                              curr_rel_target.position.z};
  std::array<float, 3> curr_vec{curr_vec_ei.x(), curr_vec_ei.y(), curr_vec_ei.z()};
  float curr_error = curr_vec_ei.norm();

  // Relative rotation error to current target pose
  Eigen::Quaternionf q_c{curr_rel_target.orientation.w, curr_rel_target.orientation.x,
                         curr_rel_target.orientation.y, curr_rel_target.orientation.z};
  Eigen::AngleAxisf aa_c{q_c};
  float curr_angle_error = std::min(aa_c.angle(), (float)(M_PI / 6));  // [0, 30 degrees]

  // Movement state machine, output delayed by one clock cycle
  switch (movement_state_) {
    case MovementState::START: {
      // Startpoint behaviour [state=0]: Go to the beginning of defined path. V = E
      velocity = p_control_velocity(curr_vec, max_speed_, 1);

      if (debug_) print_debug_info(logger, velocity);

      if (curr_error <= startpoint_tolerance_) {
        movement_state_ = MovementState::TRAVERSE;
      }
      return velocity;
      break;
    }

    case MovementState::TRAVERSE: {
      // Regular path traversal [state=1]: Find closest point along path (strictly increasing) and
      // the next point, then compute tangent velocity. Add closest point error vector. V = E + T

      // Get next target pose on the path relative to robot
      geometry_msgs::msg::Pose next_rel_target;
      geometry_msgs::msg::Pose next_target{*next_path_itr_};
      tf2::doTransform(next_target, next_rel_target, world2robot_t_);

      // Relative [x, y, z] vector to next target pose
      Eigen::Vector3f next_vec_ei{next_rel_target.position.x, next_rel_target.position.y,
                                  next_rel_target.position.z};
      std::array<float, 3> next_vec{next_vec_ei.x(), next_vec_ei.y(), next_vec_ei.z()};
      float next_error = next_vec_ei.norm();

      // Relative rotation error to next target pose
      Eigen::Quaternionf q_n{next_rel_target.orientation.w, next_rel_target.orientation.x,
                             next_rel_target.orientation.y, next_rel_target.orientation.z};
      Eigen::AngleAxisf aa_n{q_n};
      float next_angle_error = std::min(aa_n.angle(), (float)(M_PI / 6));  // [0, 30 degrees]

      // If next target is closer to robot and current orientation has been hit, increment current
      // target to the next
      while (((next_error - curr_error) <= 0.001) &&
             (curr_angle_error < (M_PI / 12))) {  // angle error < 15 degrees
        current_path_itr_++;
        next_path_itr_++;
        curr_rel_target = next_rel_target;
        curr_vec = next_vec;
        curr_error = next_error;
        curr_angle_error = next_angle_error;

        // Endpoint behaviour [state=2]: Zero in on last pose. V = E
        if (next_path_itr_ == path_.end()) {
          velocity = p_control_velocity(curr_vec, max_speed_, 1);

          if (debug_) print_debug_info(logger, velocity);

          movement_state_ = MovementState::END;
          return velocity;
        }

        // Get next target pose on the path relative to robot
        next_target = *next_path_itr_;
        tf2::doTransform(next_target, next_rel_target, world2robot_t_);

        // Relative [x, y, z] vector to next target pose
        next_vec_ei = Eigen::Vector3f{next_rel_target.position.x, next_rel_target.position.y,
                                      next_rel_target.position.z};
        next_vec = std::array<float, 3>{next_vec_ei.x(), next_vec_ei.y(), next_vec_ei.z()};
        next_error = next_vec_ei.norm();

        // Relative rotation error to next target pose
        q_n = Eigen::Quaternionf{next_rel_target.orientation.w, next_rel_target.orientation.x,
                                 next_rel_target.orientation.y, next_rel_target.orientation.z};
        aa_n = Eigen::AngleAxisf{q_n};
        next_angle_error = std::min(aa_n.angle(), (float)(M_PI / 6));  // [0, 30 degrees]
      }

      std::array<float, 3> tangent_vec{
          next_vec.at(0) - curr_vec.at(0),
          next_vec.at(1) - curr_vec.at(1),
          next_vec.at(2) - curr_vec.at(2),
      };
      velocity = traversal_velocity(tangent_vec, curr_vec, logger);

      // Scale velocity depending on relative rotation error:
      // At 15 degree error, velocity is scaled by 1/2.
      // At 30 degree error, no velocity.
      float scalar = 1 - (6 / M_PI) * curr_angle_error;
      velocity.at(0) *= scalar;
      velocity.at(1) *= scalar;
      velocity.at(2) *= scalar;

      if (debug_) print_debug_info(logger, velocity);

      return velocity;
      break;
    }

    case MovementState::END: {
      // Endpoint behaviour [state=2]: Zero in on last pose. V = E
      velocity = p_control_velocity(curr_vec, max_speed_, 1);

      if (debug_) print_debug_info(logger, velocity);

      return velocity;
      break;
    }

    default: {
      RCLCPP_ERROR(logger, "Movement state not registered, returning NAN");
      return velocity;
    }
  }
}

std::array<float, 3> Navigator::seek_angular(
    const geometry_msgs::msg::TransformStamped& world2robot_t_, rclcpp::Logger logger) {
  // Path to follow
  assert(!is_path_empty());

  // Get current (closest) target pose on the path relative to robot
  geometry_msgs::msg::Pose relative_target;
  geometry_msgs::msg::Pose current_target{*current_path_itr_};
  tf2::doTransform(current_target, relative_target, world2robot_t_);

  // Simple p-control angular velocity
  std::array<float, 3> rotation_vec{
      nav::quaternion_to_euler(relative_target.orientation.x, relative_target.orientation.y,
                               relative_target.orientation.z, relative_target.orientation.w)};

  rotation_vec = p_control_velocity(rotation_vec, max_angular_speed_, 1.5);

  return rotation_vec;
}

// V = E
std::array<float, 3> Navigator::p_control_velocity(const std::array<float, 3>& error_vec,
                                                   float max_speed, float gain) {
  Eigen::Vector3f p_vec{error_vec.at(0), error_vec.at(1), error_vec.at(2)};
  float p_vec_length = p_vec.norm();

  float speed = std::min(p_vec_length * gain, max_speed);

  p_vec.normalize();
  p_vec = p_vec * speed;

  return std::array<float, 3>{p_vec[0], p_vec[1], p_vec[2]};
}

// V = E + T
std::array<float, 3> Navigator::traversal_velocity(const std::array<float, 3>& tangent_vec,
                                                   const std::array<float, 3>& error_vec,
                                                   rclcpp::Logger logger) {
  // Calculate velocity with error and tangent vector components, V = E + T
  std::array<float, 3> velocity{0.0, 0.0, 0.0};

  // Add error velocity E (robot to closest point on path). Magnitude: [0, max_speed]
  std::array<float, 3> error_velocity = p_control_velocity(error_vec, max_speed_, 1.5);
  for (size_t i = 0; i < velocity.size(); i++) {
    velocity.at(i) += error_velocity.at(i);
  }

  Eigen::Vector3f err_vel_ei{error_velocity.at(0), error_velocity.at(1), error_velocity.at(2)};
  float error_speed = err_vel_ei.norm();

  // Add tangent velocity T (closest point on path to next point).
  // Magnitude: [0, max_speed - error_speed]
  Eigen::Vector3f tan_vel{tangent_vec.at(0), tangent_vec.at(1), tangent_vec.at(2)};
  tan_vel.normalize();
  float tangent_speed = std::sqrt(max_speed_ * max_speed_ - error_speed * error_speed);
  if (std::isnan(tangent_speed)) tangent_speed = 0.0;
  tan_vel = tan_vel * tangent_speed;

  for (size_t i = 0; i < velocity.size(); i++) {
    velocity.at(i) += tan_vel[i];
  }

  // Scale velocity depending on position on path

  float distance_remaining = get_distance_remaining();

  if (distance_remaining < goal_tolerance_) {
    // ENDING: Scale velocity magnitude proportional to final endpoint distance

    Eigen::Vector3f vel{velocity.at(0), velocity.at(1), velocity.at(2)};
    vel.normalize();
    float endpoint_speed = std::min(distance_remaining, max_speed_);
    vel = vel * endpoint_speed;

    for (size_t i = 0; i < velocity.size(); i++) {
      velocity.at(i) = vel[i];
    }
  }

  return velocity;
}

/* ------ DEBUG ------ */

void Navigator::set_debug(bool debug_flag) { debug_ = debug_flag; }

void Navigator::print_debug_info(rclcpp::Logger logger, const std::array<float, 3>& velocity) {
  float speed = sqrt(velocity.at(0) * velocity.at(0) + velocity.at(1) * velocity.at(1) +
                     velocity.at(2) * velocity.at(2));
  float remaining = get_distance_remaining();
  float travelled = get_distance_travelled();

  const char* state_name = nullptr;
  const char* vel_expr = nullptr;
  switch (movement_state_) {
    case MovementState::START:
      state_name = "STARTPOINT BEHAV";
      vel_expr = "VELOCITY V=E";
      break;
    case MovementState::TRAVERSE:
      state_name = "TRAVERSAL BEHAV";
      vel_expr = "VELOCITY V=T+E";
      break;
    case MovementState::END:
      state_name = "ENDPOINT BEHAV";
      vel_expr = "VELOCITY V=E";
      break;
    default:
      RCLCPP_WARN(logger, "DEBUG INFO INCORRECTLY SET");
      return;
  }
  RCLCPP_INFO(logger,
              "[%d] %-16s |  TRAVELLED: [ %5.2f ] REMAINING: [ %5.2f ]  |  %14s: [ %5.2f , %5.2f "
              ", %5.2f ] "
              "SPEED: [ %3.2f ] ",
              movement_state_, state_name, travelled, remaining, vel_expr, velocity.at(0),
              velocity.at(1), velocity.at(2), speed);
}
