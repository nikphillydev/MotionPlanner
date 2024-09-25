#include "at_messages/action/navigate.hpp"
#include "at_motion_planner/utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class TestServerNode : public rclcpp::Node {
 public:
  TestServerNode();

  // Send the navigation goal to the server
  void send_goal();
  void cancel_goal(
      const rclcpp_action::ClientGoalHandle<at_messages::action::Navigate>::SharedPtr& goal_handle);

 private:
  rclcpp_action::Client<at_messages::action::Navigate>::SharedPtr nav_client_;

  // Handling goal response callback
  void goal_response_callback(
      const rclcpp_action::ClientGoalHandle<at_messages::action::Navigate>::SharedPtr& goal_handle);

  // Handling feedback callback
  void feedback_callback(
      rclcpp_action::ClientGoalHandle<at_messages::action::Navigate>::SharedPtr,
      const std::shared_ptr<const at_messages::action::Navigate::Feedback> feedback);

  // Handling result callback
  void result_callback(
      const rclcpp_action::ClientGoalHandle<at_messages::action::Navigate>::WrappedResult& result);
};
