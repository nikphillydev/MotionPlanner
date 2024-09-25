#pragma once

#include <Eigen/Dense>
#include <cmath>
#include "at_messages/msg/dynamics_state.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace nav {

geometry_msgs::msg::Quaternion create_quaternion(const double& x, const double& y, const double& z,
                                                 const double& w);

geometry_msgs::msg::Point create_point(const double& x, const double& y, const double& z);

geometry_msgs::msg::Pose create_pose(const double& x, const double& y, const double& z,
                                     const double& qx, const double& qy, const double& qz,
                                     const double& qw);

std_msgs::msg::Header create_header(const std::string& frame_id,
                                    const builtin_interfaces::msg::Time& stamp);

geometry_msgs::msg::PoseStamped create_pose_stamped(const std::string& frame_id,
                                                    const builtin_interfaces::msg::Time& stamp,
                                                    const double& x, const double& y,
                                                    const double& z, const double& qx,
                                                    const double& qy, const double& qz,
                                                    const double& qw);

geometry_msgs::msg::PoseStamped create_pose_stamped(const double& x, const double& y,
                                                    const double& z, const double& qx,
                                                    const double& qy, const double& qz,
                                                    const double& qw);

Eigen::Quaterniond euler_to_quaternion(const double& roll, const double& pitch, const double& yaw);

std::array<float, 3> quaternion_to_euler(const double& x, const double& y, const double& z,
                                         const double& w);

at_messages::msg::DynamicsState dynamics_state_nan(const std::string& frame_id,
                                                   const builtin_interfaces::msg::Time& stamp);

bool equals(const geometry_msgs::msg::Pose& left, const geometry_msgs::msg::Pose& right);

}  // namespace nav
