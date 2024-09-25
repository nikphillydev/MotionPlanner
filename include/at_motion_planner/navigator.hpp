#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include "at_motion_planner/spline.hpp"
#include "at_motion_planner/utils.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

enum MovementState { START, TRAVERSE, END };

class Navigator {
 public:
  Navigator();

  /* --- Interface --- */

  bool create_path(const std::vector<geometry_msgs::msg::Pose>& targets, int poses_per_meter,
                   int orient_mode, const geometry_msgs::msg::Quaternion& orientation,
                   bool level_lock, rclcpp::Logger logger);
  void reset_path();
  bool is_path_empty() const;

  std::array<float, 6> seek_velocity(const geometry_msgs::msg::TransformStamped& world2robot_t_,
                                     rclcpp::Logger logger);
  bool reached_goal(const geometry_msgs::msg::TransformStamped& world2robot_t_,
                    rclcpp::Logger logger) const;

  /* --- Getters --- */
  // Distance
  float get_distance_remaining() const;
  float get_distance_travelled() const;
  // Rviz
  nav_msgs::msg::Path get_path() const;
  geometry_msgs::msg::PoseArray get_path_poses() const;
  geometry_msgs::msg::PoseArray get_path_targets() const;
  geometry_msgs::msg::Pose get_current_pose() const;

  /* --- Setters --- */
  void set_debug(bool debug_flag);

 private:
  /* --- Debug --- */
  bool debug_;
  void print_debug_info(rclcpp::Logger logger, const std::array<float, 3>& velocity);

  /* --- Velocity configuration --- */
  float startpoint_tolerance_;
  float goal_tolerance_;
  float angular_goal_tolerance_;
  float max_speed_;
  float max_angular_speed_;

  /* --- Path definitions --- */
  MovementState movement_state_;
  float path_length_;
  std::vector<geometry_msgs::msg::Pose> targets_;
  std::vector<geometry_msgs::msg::Pose> path_;
  std::vector<geometry_msgs::msg::Pose>::const_iterator current_path_itr_;
  std::vector<geometry_msgs::msg::Pose>::const_iterator next_path_itr_;
  bool itr_ready_;

  /* --- Internal velocity calculation --- */
  std::array<float, 3> seek_linear(const geometry_msgs::msg::TransformStamped& world2robot_t_,
                                   rclcpp::Logger logger);
  std::array<float, 3> seek_angular(const geometry_msgs::msg::TransformStamped& world2robot_t_,
                                    rclcpp::Logger logger);
  std::array<float, 3> p_control_velocity(const std::array<float, 3>& error_vec, float max_speed,
                                          float gain);
  std::array<float, 3> traversal_velocity(const std::array<float, 3>& tangent_vec,
                                          const std::array<float, 3>& error_vec,
                                          rclcpp::Logger logger);
};
