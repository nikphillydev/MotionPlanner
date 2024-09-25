#include <thread>
#include "at_messages/action/navigate.hpp"
#include "at_messages/msg/dynamics_state.hpp"
#include "at_motion_planner/navigator.hpp"
#include "at_motion_planner/utils.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class MotionPlannerNode : public rclcpp::Node {
 public:
  MotionPlannerNode();

  /* --- Callbacks --- */
  void robot_transform_timer_cb();
  void rviz_path_timer_cb() const;
  void waypoint_cb(const geometry_msgs::msg::PoseStamped& ps);

  /* --- Utility methods --- */
  void update_feedback(std::shared_ptr<at_messages::action::Navigate::Feedback> feedback,
                       const std::array<float, 6>& velocity) const;
  geometry_msgs::msg::Pose transform_pose(const geometry_msgs::msg::Pose& robot_pose,
                                          const geometry_msgs::msg::PoseStamped& t_ps) const;
  geometry_msgs::msg::Pose transform_pose_to_world(const geometry_msgs::msg::PoseStamped& ps) const;

 private:
  Navigator navigator_;

  /* --- Pointer to waypoint subsciption pose --- */
  std::unique_ptr<geometry_msgs::msg::Pose> waypoint_pose_ptr;

  /* --- TF2 --- */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  geometry_msgs::msg::TransformStamped world2robot_t_;
  geometry_msgs::msg::TransformStamped robot2world_t_;

  /* --- Timers --- */
  // Robot transform update
  rclcpp::TimerBase::SharedPtr robot_transform_timer_;
  // RViz visualization
  rclcpp::TimerBase::SharedPtr rviz_path_timer_;

  /* --- Publishers --- */
  // Rviz visualization
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr targets_pub_;
  // Target state
  rclcpp::Publisher<at_messages::msg::DynamicsState>::SharedPtr target_state_pub_;

  /* --- Callback group for action server --- */
  rclcpp::CallbackGroup::SharedPtr nav_server_cb_group_;

  /* --- Action server --- */
  rclcpp_action::Server<at_messages::action::Navigate>::SharedPtr nav_server_;

  /* --- Handling goal callback --- */
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const at_messages::action::Navigate::Goal> goal);

  /* --- Handling cancellation callback --- */
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<at_messages::action::Navigate>>
          goal_handle);

  /* --- Handling goal accept callback --- */
  void handle_accept(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<at_messages::action::Navigate>>
          goal_handle);

  /* --- Execute navigation goal --- */
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<at_messages::action::Navigate>>
                   goal_handle);
};
