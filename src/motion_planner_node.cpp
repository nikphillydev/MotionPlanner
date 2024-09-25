#include "at_motion_planner/motion_planner_node.hpp"

MotionPlannerNode::MotionPlannerNode()
    : Node{"motion_planner_node"}, navigator_{}, waypoint_pose_ptr{nullptr} {
  // TF2
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/motion/path", 10);
  poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/motion/poses", 10);
  targets_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/motion/targets", 10);
  target_state_pub_ =
      this->create_publisher<at_messages::msg::DynamicsState>("/dynamics/target", 10);

  // Timers
  rviz_path_timer_ =
      this->create_wall_timer(50ms, std::bind(&MotionPlannerNode::rviz_path_timer_cb, this));
  robot_transform_timer_ =
      this->create_wall_timer(20ms, std::bind(&MotionPlannerNode::robot_transform_timer_cb, this));

  // Callback group
  nav_server_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Action server
  nav_server_ = rclcpp_action::create_server<at_messages::action::Navigate>(
      this, "/navigate_server", std::bind(&MotionPlannerNode::handle_goal, this, _1, _2),
      std::bind(&MotionPlannerNode::handle_cancel, this, _1),
      std::bind(&MotionPlannerNode::handle_accept, this, _1),
      rcl_action_server_get_default_options(), nav_server_cb_group_);

  RCLCPP_INFO(this->get_logger(), "Motion planner node started...");
  RCLCPP_INFO(this->get_logger(), "----------------------------------");
}

/* ------ Robot transform update and Rviz path publishers (default callback group) ------ */

void MotionPlannerNode::robot_transform_timer_cb() {
  std::string robot_frame{"base_link_ned"};
  std::string world_frame{"world"};

  try {
    world2robot_t_ = tf_buffer_->lookupTransform(robot_frame, world_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform %s to %s (only an issue if failure repeats: %s",
                 world_frame.c_str(), robot_frame.c_str(), ex.what());
  }
  try {
    robot2world_t_ = tf_buffer_->lookupTransform(world_frame, robot_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform %s to %s (only an issue if failure repeats: %s",
                 robot_frame.c_str(), world_frame.c_str(), ex.what());
  }
}

void MotionPlannerNode::rviz_path_timer_cb() const {
  // The "smooth" path
  nav_msgs::msg::Path nav_path{navigator_.get_path()};
  path_pub_->publish(nav_path);

  // Poses along the path
  geometry_msgs::msg::PoseArray poses{navigator_.get_path_poses()};
  poses_pub_->publish(poses);

  // Main targets on path
  geometry_msgs::msg::PoseArray targets{navigator_.get_path_targets()};
  targets_pub_->publish(targets);
}

/* ------ Action server callbacks (nav server callback group) ------ */

rclcpp_action::GoalResponse MotionPlannerNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const at_messages::action::Navigate::Goal> goal) {
  RCLCPP_INFO(this->get_logger(),
              "Received goal request with target_mode: [ %d ] and orient_mode: [ %d ]",
              goal->target_mode, goal->orient_mode);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionPlannerNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<at_messages::action::Navigate>>
        goal_handle) {
  RCLCPP_WARN(this->get_logger(), "Received request to cancel goal...");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionPlannerNode::handle_accept(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<at_messages::action::Navigate>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Accepting goal...");
  // Execute navigation in a separate thread to avoid blocking executor from accepting cancel
  // callbacks
  std::thread{std::bind(&MotionPlannerNode::execute, this, _1), goal_handle}.detach();
}

/* ------ Action server execution (separate thread) ------ */

void MotionPlannerNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<at_messages::action::Navigate>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(), "----------------------------------");
  RCLCPP_INFO(this->get_logger(), "Starting execution of goal");

  const std::shared_ptr<const at_messages::action::Navigate::Goal> goal = goal_handle->get_goal();
  std::shared_ptr<at_messages::action::Navigate::Feedback> feedback =
      std::make_shared<at_messages::action::Navigate::Feedback>();
  std::shared_ptr<at_messages::action::Navigate::Result> result =
      std::make_shared<at_messages::action::Navigate::Result>();

  /* --- 1. Create the target poses for navigator --- */

  RCLCPP_INFO(this->get_logger(), "Initializing target poses for target_mode = %d ...",
              goal->target_mode);

  std::vector<geometry_msgs::msg::Pose> poses{};

  geometry_msgs::msg::Pose current_pose{nav::create_pose(
      robot2world_t_.transform.translation.x, robot2world_t_.transform.translation.y,
      robot2world_t_.transform.translation.z, robot2world_t_.transform.rotation.x,
      robot2world_t_.transform.rotation.y, robot2world_t_.transform.rotation.z,
      robot2world_t_.transform.rotation.w)};
  poses.push_back(current_pose);

  // Waypoint topic subscription
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
          goal->waypoint_topic, 10, std::bind(&MotionPlannerNode::waypoint_cb, this, _1));

  // Initialize robot target positions
  switch (goal->target_mode) {
    case 0: {
      RCLCPP_INFO(this->get_logger(), "\t-Transform targets to world frame");
      for (size_t i = 0; i < goal->poses.size(); i++) {
        poses.push_back(transform_pose_to_world(goal->poses.at(i)));
      }
      break;
    }

    case 1: {
      RCLCPP_INFO(this->get_logger(), "\t-Listen for target on waypoint topic");
      rclcpp::Rate waypoint_rate(1s);
      while (!waypoint_pose_ptr) {
        RCLCPP_WARN(this->get_logger(), "Target on waypoint topic: %s not received, waiting ...",
                    goal->waypoint_topic.c_str());
        waypoint_rate.sleep();
      }
      poses.push_back(*waypoint_pose_ptr);
      break;
    }

    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid target mode. Aborting navigation");
      result->success = false;
      goal_handle->abort(result);
      return;
  }

  // Adjust targets so requested robot frame moves through the targets
  for (size_t i = 1; i < poses.size(); i++) {
    geometry_msgs::msg::TransformStamped robot2subsystem_t;
    try {
      robot2subsystem_t =
          tf_buffer_->lookupTransform(goal->robot_frame, "base_link_ned", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform robot to %s: %s",
                   goal->robot_frame.c_str(), ex.what());
    }
    geometry_msgs::msg::Pose subsystemInTarget_pose;
    subsystemInTarget_pose.position.x = robot2subsystem_t.transform.translation.x;
    subsystemInTarget_pose.position.y = robot2subsystem_t.transform.translation.y;
    subsystemInTarget_pose.position.z = robot2subsystem_t.transform.translation.z;
    subsystemInTarget_pose.orientation.x = robot2subsystem_t.transform.rotation.x;
    subsystemInTarget_pose.orientation.y = robot2subsystem_t.transform.rotation.y;
    subsystemInTarget_pose.orientation.z = robot2subsystem_t.transform.rotation.z;
    subsystemInTarget_pose.orientation.w = robot2subsystem_t.transform.rotation.w;

    geometry_msgs::msg::TransformStamped target2world_t;
    target2world_t.transform.translation.x = poses.at(i).position.x;
    target2world_t.transform.translation.y = poses.at(i).position.y;
    target2world_t.transform.translation.z = poses.at(i).position.z;
    target2world_t.transform.rotation.x = poses.at(i).orientation.x;
    target2world_t.transform.rotation.y = poses.at(i).orientation.y;
    target2world_t.transform.rotation.z = poses.at(i).orientation.z;
    target2world_t.transform.rotation.w = poses.at(i).orientation.w;

    geometry_msgs::msg::Pose subsystemInWorld_pose;
    tf2::doTransform(subsystemInTarget_pose, subsystemInWorld_pose, target2world_t);
    poses.at(i) = subsystemInWorld_pose;
  }

  RCLCPP_INFO(this->get_logger(), "Target poses initialized and ready for path creation");

  /* --- 2. Create the path --- */

  RCLCPP_INFO(this->get_logger(), "Creating the path");

  geometry_msgs::msg::Quaternion quat = goal->constant_orientation;
  if (goal->orient_mode == 0) {
    quat = goal->offset_orientation;
  }
  bool path_ok = navigator_.create_path(poses, 50, goal->orient_mode, quat, goal->level_lock,
                                        this->get_logger());
  if (!path_ok) {
    RCLCPP_ERROR(this->get_logger(), "Path creation failed. Aborting navigation");
    result->success = false;
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Path creation success");

  /* --- 3. Navigate the path --- */

  RCLCPP_INFO(this->get_logger(), "Beginning high frequency path traversal");

  rclcpp::Rate loop_rate(50ms);  // Path traversal at 20Hz

  while (!navigator_.reached_goal(world2robot_t_, this->get_logger())) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->success = true;
      goal_handle->canceled(result);
      RCLCPP_WARN(this->get_logger(), "Goal canceled");
      RCLCPP_INFO(this->get_logger(), "----------------------------------");
      return;
    }

    // Get target velocity
    std::array<float, 6> velocity{navigator_.seek_velocity(world2robot_t_, this->get_logger())};

    // Check velocity (linear and angular) validity
    if (std::isnan(velocity.at(0)) || std::isnan(velocity.at(1)) || std::isnan(velocity.at(2))) {
      RCLCPP_ERROR(this->get_logger(), "Linear velocity returned is NAN");
    }
    if (std::isnan(velocity.at(3)) || std::isnan(velocity.at(4)) || std::isnan(velocity.at(5))) {
      RCLCPP_ERROR(this->get_logger(), "Angular velocity returned is NAN");
    }

    // Create target state velocity
    at_messages::msg::DynamicsState target_state{
        nav::dynamics_state_nan("world", this->get_clock()->now())};
    target_state.velocity.at(0) = velocity.at(0);  // Linear x
    target_state.velocity.at(1) = velocity.at(1);  // Linear y
    target_state.velocity.at(2) = velocity.at(2);  // Linear z
    target_state.velocity.at(3) = velocity.at(3);  // Roll
    target_state.velocity.at(4) = velocity.at(4);  // Pitch
    target_state.velocity.at(5) = velocity.at(5);  // Yaw

    // Send target velocity
    target_state_pub_->publish(target_state);

    // Send feedback to client
    update_feedback(feedback, velocity);
    goal_handle->publish_feedback(feedback);

    // Create a new path with subscribed waypoint, if necessary
    if (goal->target_mode == 1 && !navigator_.reached_goal(world2robot_t_, this->get_logger())) {
      current_pose = navigator_.get_current_pose();
      poses = std::vector<geometry_msgs::msg::Pose>{};
      poses.push_back(current_pose);
      poses.push_back(*waypoint_pose_ptr);

      // Adjust targets so robot moves through requested robot frame
      for (size_t i = 1; i < poses.size(); i++) {
        geometry_msgs::msg::TransformStamped robot2subsystem_t;
        try {
          robot2subsystem_t =
              tf_buffer_->lookupTransform(goal->robot_frame, "base_link_ned", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
          RCLCPP_ERROR(this->get_logger(), "Could not transform robot to claw: %s", ex.what());
        }
        geometry_msgs::msg::Pose subsystemInTarget_pose;
        subsystemInTarget_pose.position.x = robot2subsystem_t.transform.translation.x;
        subsystemInTarget_pose.position.y = robot2subsystem_t.transform.translation.y;
        subsystemInTarget_pose.position.z = robot2subsystem_t.transform.translation.z;
        subsystemInTarget_pose.orientation.x = robot2subsystem_t.transform.rotation.x;
        subsystemInTarget_pose.orientation.y = robot2subsystem_t.transform.rotation.y;
        subsystemInTarget_pose.orientation.z = robot2subsystem_t.transform.rotation.z;
        subsystemInTarget_pose.orientation.w = robot2subsystem_t.transform.rotation.w;

        geometry_msgs::msg::TransformStamped target2world_t;
        target2world_t.transform.translation.x = poses.at(i).position.x;
        target2world_t.transform.translation.y = poses.at(i).position.y;
        target2world_t.transform.translation.z = poses.at(i).position.z;
        target2world_t.transform.rotation.x = poses.at(i).orientation.x;
        target2world_t.transform.rotation.y = poses.at(i).orientation.y;
        target2world_t.transform.rotation.z = poses.at(i).orientation.z;
        target2world_t.transform.rotation.w = poses.at(i).orientation.w;

        geometry_msgs::msg::Pose subsystemInWorld_pose;
        tf2::doTransform(subsystemInTarget_pose, subsystemInWorld_pose, target2world_t);
        poses.at(i) = subsystemInWorld_pose;
      }

      geometry_msgs::msg::Quaternion quat = goal->constant_orientation;
      if (goal->orient_mode == 0) {
        quat = goal->offset_orientation;
      }
      bool path_ok = navigator_.create_path(poses, 50, goal->orient_mode, quat, goal->level_lock,
                                            this->get_logger());
      if (!path_ok) {
        RCLCPP_ERROR(this->get_logger(), "Path creation failed. Aborting navigation");
        result->success = false;
        goal_handle->abort(result);
        return;
      }
    }

    loop_rate.sleep();
  }

  if (goal->end_as_final_target) {
    // Create the path from the current position to the last pose, interpolated orientation
    current_pose = nav::create_pose(
        robot2world_t_.transform.translation.x, robot2world_t_.transform.translation.y,
        robot2world_t_.transform.translation.z, robot2world_t_.transform.rotation.x,
        robot2world_t_.transform.rotation.y, robot2world_t_.transform.rotation.z,
        robot2world_t_.transform.rotation.w);
    poses = std::vector<geometry_msgs::msg::Pose>{};
    poses.push_back(current_pose);
    poses.push_back(poses.back());
    geometry_msgs::msg::Quaternion placeholder_q{nav::create_quaternion(0, 0, 0, 1)};
    bool path_ok =
        navigator_.create_path(poses, 25, 50, placeholder_q, goal->level_lock, this->get_logger());
    if (!path_ok) {
      RCLCPP_ERROR(this->get_logger(), "Path creation failed. Aborting navigation");
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    // Move robot to last pose position and orientation
    while (!navigator_.reached_goal(world2robot_t_, this->get_logger())) {
      // Get target velocity
      std::array<float, 6> velocity{navigator_.seek_velocity(world2robot_t_, this->get_logger())};

      // Check velocity (linear and angular) validity
      if (std::isnan(velocity.at(0)) || std::isnan(velocity.at(1)) || std::isnan(velocity.at(2))) {
        RCLCPP_ERROR(this->get_logger(), "Linear velocity returned is NAN");
      }
      if (std::isnan(velocity.at(3)) || std::isnan(velocity.at(4)) || std::isnan(velocity.at(5))) {
        RCLCPP_ERROR(this->get_logger(), "Angular velocity returned is NAN");
      }

      // Create target state velocity
      at_messages::msg::DynamicsState target_state{
          nav::dynamics_state_nan("world", this->get_clock()->now())};
      target_state.velocity.at(0) = velocity.at(0);  // Linear x
      target_state.velocity.at(1) = velocity.at(1);  // Linear y
      target_state.velocity.at(2) = velocity.at(2);  // Linear z
      target_state.velocity.at(3) = velocity.at(3);  // Roll
      target_state.velocity.at(4) = velocity.at(4);  // Pitch
      target_state.velocity.at(5) = velocity.at(5);  // Yaw

      // Send target velocity
      target_state_pub_->publish(target_state);

      // Send feedback to client
      update_feedback(feedback, velocity);
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    at_messages::msg::DynamicsState target_state{
        nav::dynamics_state_nan("world", this->get_clock()->now())};
    target_state.position.at(0) = poses.back().position.x;
    target_state.position.at(1) = poses.back().position.y;
    target_state.position.at(2) = poses.back().position.z;
    std::array<float, 3> euler{
        nav::quaternion_to_euler(poses.back().orientation.x, poses.back().orientation.y,
                                 poses.back().orientation.z, poses.back().orientation.w)};
    target_state.position.at(3) = euler.at(0);
    target_state.position.at(4) = euler.at(1);
    target_state.position.at(5) = euler.at(2);
    target_state_pub_->publish(target_state);
  }

  /* --- 4. Reset state --- */

  navigator_.reset_path();
  waypoint_pose_ptr = nullptr;

  // Return success to client
  result->success = true;
  goal_handle->succeed(result);

  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  RCLCPP_INFO(this->get_logger(), "----------------------------------");
}

void MotionPlannerNode::waypoint_cb(const geometry_msgs::msg::PoseStamped& ps) {
  waypoint_pose_ptr = std::make_unique<geometry_msgs::msg::Pose>();
  *waypoint_pose_ptr = ps.pose;
}

void MotionPlannerNode::update_feedback(
    std::shared_ptr<at_messages::action::Navigate::Feedback> feedback,
    const std::array<float, 6>& velocity) const {
  float travelled = navigator_.get_distance_travelled();
  float remaining = navigator_.get_distance_remaining();
  float linear_speed = sqrt(velocity.at(0) * velocity.at(0) + velocity.at(1) * velocity.at(1) +
                            velocity.at(2) * velocity.at(2));
  float angular_speed = sqrt(velocity.at(3) * velocity.at(3) + velocity.at(4) * velocity.at(4) +
                             velocity.at(5) * velocity.at(5));

  feedback->travelled = travelled;
  feedback->remaining = remaining;
  feedback->linear_speed = linear_speed;
  feedback->angular_speed = angular_speed;
  RCLCPP_INFO(this->get_logger(), "%f hello_world", angular_speed);
}

// rel_ps: pose relative to an arbitrary robot frame (either robot frame or child frame of robot).
// robot_p: arbitrary robot pose (in world frame).
// Converts rel_ps to world frame while maintaining its relative position to the arbitrary robot
// pose
geometry_msgs::msg::Pose MotionPlannerNode::transform_pose(
    const geometry_msgs::msg::Pose& robot_p, const geometry_msgs::msg::PoseStamped& rel_ps) const {
  std::string robot_frame{"base_link_ned"};
  std::string rel_ps_frame{rel_ps.header.frame_id};

  geometry_msgs::msg::Pose rel_p{rel_ps.pose};

  if (rel_ps_frame != robot_frame) {
    // Create transform from rel_ps to robot_frame
    geometry_msgs::msg::TransformStamped rel2robot_t;
    try {
      rel2robot_t = tf_buffer_->lookupTransform(robot_frame, rel_ps_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s", rel_ps_frame.c_str(),
                   robot_frame.c_str(), ex.what());
      RCLCPP_WARN(this->get_logger(), "Returning pose value from frame: %s", rel_ps_frame.c_str());
      return rel_ps.pose;
    }

    // Apply transform to get rel_ps in robot frame
    tf2::doTransform(rel_ps.pose, rel_p, rel2robot_t);
  }

  // Create transform from robot_p to world
  geometry_msgs::msg::TransformStamped arb_robot2world_t;
  arb_robot2world_t.header.stamp = this->get_clock()->now();
  arb_robot2world_t.header.frame_id = "arb_robot";
  arb_robot2world_t.child_frame_id = "world";
  arb_robot2world_t.transform.translation.x = robot_p.position.x;
  arb_robot2world_t.transform.translation.y = robot_p.position.y;
  arb_robot2world_t.transform.translation.z = robot_p.position.z;
  arb_robot2world_t.transform.rotation.x = robot_p.orientation.x;
  arb_robot2world_t.transform.rotation.y = robot_p.orientation.y;
  arb_robot2world_t.transform.rotation.z = robot_p.orientation.z;
  arb_robot2world_t.transform.rotation.w = robot_p.orientation.w;

  // Apply transform to get rel_ps relative to the robot in world frame
  geometry_msgs::msg::Pose world_p{};
  tf2::doTransform(rel_p, world_p, arb_robot2world_t);

  return world_p;
}

geometry_msgs::msg::Pose MotionPlannerNode::transform_pose_to_world(
    const geometry_msgs::msg::PoseStamped& ps) const {
  std::string ps_frame{ps.header.frame_id};
  std::string world_frame{"world"};

  if (ps_frame == world_frame) {
    // No transform needed
    return ps.pose;
  }

  // Create transform from ps_frame to world
  geometry_msgs::msg::TransformStamped toWorld_t;
  try {
    toWorld_t = tf_buffer_->lookupTransform(world_frame, ps_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(this->get_logger(), "Could not transform %s to world: %s", ps_frame.c_str(),
                 ex.what());
    RCLCPP_WARN(this->get_logger(), "Returning pose value from frame: %s", ps_frame.c_str());
    return ps.pose;
  }

  // Apply transform to get ps in world frame
  geometry_msgs::msg::Pose world_p;
  tf2::doTransform(ps.pose, world_p, toWorld_t);

  return world_p;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<MotionPlannerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
