#include "at_motion_planner/test_server_node.hpp"

TestServerNode::TestServerNode() : Node("test_server_node") {
  nav_client_ =
      rclcpp_action::create_client<at_messages::action::Navigate>(this, "/navigate_server");

  RCLCPP_INFO(this->get_logger(), "Test navigation server node started...");

  send_goal();
}

void TestServerNode::send_goal() {
  while (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Navigation server not available, waiting...");
  }

  at_messages::action::Navigate::Goal goal_msg = at_messages::action::Navigate::Goal();
  // --------------------------------------------------------------------------------
  // Goal definition ----------------------------------------------------------------
  // --------------------------------------------------------------------------------

  // Linear velocity mode
  goal_msg.target_mode = 0;

  // some example movement types for testing ...

  // ------------------- "DO A BARREL ROLL!" --------------------
  // geometry_msgs::msg::PoseStamped ps1{};
  // ps1 = nav::create_pose_stamped(0.5, 0.0, 1.0, 0, 0, 0, 1);  // RPY=000
  // geometry_msgs::msg::PoseStamped ps2{};
  // ps2 = nav::create_pose_stamped(1.5, 0.0, 1.0, 0.7071068, 0, 0, 0.7071068);
  // geometry_msgs::msg::PoseStamped ps3{};
  // ps3 = nav::create_pose_stamped(2.5, 0.0, 1.0, 1, 0, 0, 0);
  // geometry_msgs::msg::PoseStamped ps4{};
  // ps4 = nav::create_pose_stamped(3.5, 0.0, 1.0, 0.7071068, 0, 0, -0.7071068);
  // geometry_msgs::msg::PoseStamped ps5{};
  // ps5 = nav::create_pose_stamped(4.5, 0.0, 1.0, 0, 0, 0, -1);  // should be RPY=000
  // std::vector<geometry_msgs::msg::PoseStamped> targets{ps1, ps2, ps3, ps4, ps5};
  // ------------------------------------------------------------

  // ------------------ Relative motion testing -----------------
  geometry_msgs::msg::PoseStamped ps1{};
  ps1 = nav::create_pose_stamped(0, 0.0, 0.0, 0, 0, 0, 1);  // world frame
  ps1.header.frame_id = "buoy_front";                       // robot 1m forward
  // geometry_msgs::msg::PoseStamped ps2{};
  // ps2 = nav::create_pose_stamped(1.0, 0.0, 0, 0, 0, 0, 1);
  // ps2.header.frame_id = "base_link_ned";  // robot 1m forward
  // geometry_msgs::msg::PoseStamped ps3{};
  // ps3 = nav::create_pose_stamped(0.0, 1.0, 0.0, 0, 0, 0.7071068, 0.7071068);
  // ps3.header.frame_id = "base_link_ned";  // robot 1m right, turn 90deg right
  // geometry_msgs::msg::PoseStamped ps4{};
  // ps4 = nav::create_pose_stamped(-1.0, 0.0, 1.0, 0, 0, -0.7071068, 0.7071068);
  // ps4.header.frame_id = "base_link_ned";  // robot 1m backward and 1m down, turn 90deg left
  // geometry_msgs::msg::PoseStamped ps5{};
  // ps5 = nav::create_pose_stamped(1.0, -1.0, 0.0, 0, 0, 0, 1);
  // ps5.header.frame_id = "DVL";  // relative to DVL, 1m forward and 1m left
  // geometry_msgs::msg::PoseStamped ps6{};
  // ps6 = nav::create_pose_stamped(0.5, 0.0, 1.0, 0, 0, 0, 1);  // world frame, reset to initial
  std::vector<geometry_msgs::msg::PoseStamped> targets{ps1};
  // ------------------------------------------------------------

  // --------------------- The "CUBE" path ----------------------
  // geometry_msgs::msg::PoseStamped ps1{};
  // ps1 = nav::create_pose_stamped(0.0, 0.0, 1.0, 0.0, 0.0, 0.707, 0.707);
  // geometry_msgs::msg::PoseStamped ps2{};
  // ps2 = nav::create_pose_stamped(1.0, 1.0, 1.0, 0.7071068, 0, 0, 0.7071068);
  // geometry_msgs::msg::PoseStamped ps3{};
  // ps3 = nav::create_pose_stamped(3.0, 1.0, 1.0, 1, 0, 0, 0);
  // geometry_msgs::msg::PoseStamped ps4{};
  // ps4 = nav::create_pose_stamped(3.0, 3.0, 1.0, 0.7071068, 0, 0, -0.7071068);
  // geometry_msgs::msg::PoseStamped ps5{};
  // ps5 = nav::create_pose_stamped(1.0, 3.0, 1.0, 0, 0, 0, 1);
  // geometry_msgs::msg::PoseStamped ps6{};
  // ps6 = nav::create_pose_stamped(1.0, 1.0, 1.0, 0, 1, 0, 0);
  // geometry_msgs::msg::PoseStamped ps7{};
  // ps7 = nav::create_pose_stamped(1.0, 1.0, 3.0, 0, 1, 0, 0);
  // geometry_msgs::msg::PoseStamped ps8{};
  // ps8 = nav::create_pose_stamped(1.0, 3.0, 3.0, 0, 0, 0, 1);
  // geometry_msgs::msg::PoseStamped ps9{};
  // ps9 = nav::create_pose_stamped(3.0, 3.0, 3.0, 0, 0, 0, 1);
  // geometry_msgs::msg::PoseStamped ps10{};
  // ps10 = nav::create_pose_stamped(3.0, 1.0, 3.0, 0.0, 0.0, 0.0, 1.0);
  // geometry_msgs::msg::PoseStamped ps11{};
  // ps11 = nav::create_pose_stamped(1.0, 1.0, 3.0, 0.5, -0.5, 0.5, -0.5);
  // geometry_msgs::msg::PoseStamped ps12{};
  // ps12 = nav::create_pose_stamped(1.0, 1.0, 1.0, 0.0, 0.0, 0.707, 0.707);
  // std::vector<geometry_msgs::msg::PoseStamped> targets{ps1, ps2, ps3, ps4, ps5};
  // ------------------------------------------------------------

  // ------------------- Stationary Test -----------------------
  // geometry_msgs::msg::PoseStamped ps1{};
  // ps1 = nav::create_pose_stamped(0.0, 0.0, 1.0, 0, 0, 0, 1);  // RPY=000
  // geometry_msgs::msg::PoseStamped ps2{};
  // ps2 = nav::create_pose_stamped(0.0, 0.0, 1.0, 0, 0, 0.7071068, 0.7071068);
  // geometry_msgs::msg::PoseStamped ps3{};
  // ps3 = nav::create_pose_stamped(0.0, 0.0, 1.0, 0, 0, 1, 0);
  // geometry_msgs::msg::PoseStamped ps4{};
  // ps4 = nav::create_pose_stamped(0.0, 0.0, 1.0, 0, 0, 0.7071068, -0.7071068);
  // geometry_msgs::msg::PoseStamped ps5{};
  // ps5 = nav::create_pose_stamped(0.0, 0.0, 1.0, 0, 0, 0, 1);
  // std::vector<geometry_msgs::msg::PoseStamped> targets{ps1, ps2, ps3, ps4, ps5};
  // ------------------------------------------------------------

  goal_msg.poses = targets;

  goal_msg.robot_frame = "Claw_Center";

  goal_msg.waypoint_topic = "/mapping/waypoints/buoy_front";

  // Angular velocity mode
  goal_msg.orient_mode = 2;

  geometry_msgs::msg::Quaternion orientation{};
  orientation.x = 0.0;
  orientation.y = 0.0;
  orientation.z = 0.0;
  orientation.w = 1.0;
  goal_msg.offset_orientation = orientation;
  goal_msg.constant_orientation = orientation;

  goal_msg.level_lock = false;
  goal_msg.end_as_final_target = true;

  // --------------------------------------------------------------------------------
  // End goal definition ------------------------------------------------------------
  // --------------------------------------------------------------------------------

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  rclcpp_action::Client<at_messages::action::Navigate>::SendGoalOptions send_goal_options =
      rclcpp_action::Client<at_messages::action::Navigate>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&TestServerNode::goal_response_callback, this, _1);
  send_goal_options.feedback_callback = std::bind(&TestServerNode::feedback_callback, this, _1, _2);
  send_goal_options.result_callback = std::bind(&TestServerNode::result_callback, this, _1);
  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void TestServerNode::goal_response_callback(
    const rclcpp_action::ClientGoalHandle<at_messages::action::Navigate>::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    // std::thread{std::bind(&TestServerNode::cancel_goal, this, _1), goal_handle}.detach();
  }
}

void TestServerNode::cancel_goal(
    const rclcpp_action::ClientGoalHandle<at_messages::action::Navigate>::SharedPtr& goal_handle) {
  rclcpp::Rate loop_rate_{1s};
  int end = 10;
  int counter = 0;
  while (counter < end) {
    loop_rate_.sleep();
    counter++;
    RCLCPP_INFO(this->get_logger(), "Sleeping %d ...", counter);
  }
  nav_client_->async_cancel_goal(goal_handle);
}

void TestServerNode::feedback_callback(
    rclcpp_action::ClientGoalHandle<at_messages::action::Navigate>::SharedPtr,
    const std::shared_ptr<const at_messages::action::Navigate::Feedback> feedback) {
  RCLCPP_INFO(
      this->get_logger(),
      "\nNavigation status:\n\tTravelled: [ %.2f ]\n\tRemaining: [ %.2f ]\n\tLinear speed: [ "
      "%.2f ]\n\tAngular speed: [ %.2f ]",
      feedback->travelled, feedback->remaining, feedback->linear_speed, feedback->angular_speed);
}

void TestServerNode::result_callback(
    const rclcpp_action::ClientGoalHandle<at_messages::action::Navigate>::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal finished successfully");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  rclcpp::shutdown();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<TestServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
