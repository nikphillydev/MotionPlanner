#include "at_motion_planner/utils.hpp"

geometry_msgs::msg::Quaternion nav::create_quaternion(const double& x, const double& y,
                                                      const double& z, const double& w) {
  geometry_msgs::msg::Quaternion quat;
  quat.set__w(w);
  quat.set__x(x);
  quat.set__y(y);
  quat.set__z(z);
  return quat;
}

geometry_msgs::msg::Point nav::create_point(const double& x, const double& y, const double& z) {
  geometry_msgs::msg::Point pt;
  pt.set__x(x);
  pt.set__y(y);
  pt.set__z(z);
  return pt;
}

geometry_msgs::msg::Pose nav::create_pose(const double& x, const double& y, const double& z,
                                          const double& qx, const double& qy, const double& qz,
                                          const double& qw) {
  geometry_msgs::msg::Pose pose;
  pose.set__orientation(nav::create_quaternion(qx, qy, qz, qw));
  pose.set__position(nav::create_point(x, y, z));
  return pose;
}

std_msgs::msg::Header nav::create_header(const std::string& frame_id,
                                         const builtin_interfaces::msg::Time& stamp) {
  std_msgs::msg::Header header;
  header.set__frame_id(frame_id);
  header.set__stamp(stamp);
  return header;
}

geometry_msgs::msg::PoseStamped nav::create_pose_stamped(const std::string& frame_id,
                                                         const builtin_interfaces::msg::Time& stamp,
                                                         const double& x, const double& y,
                                                         const double& z, const double& qx,
                                                         const double& qy, const double& qz,
                                                         const double& qw) {
  geometry_msgs::msg::PoseStamped ps;
  ps.set__header(nav::create_header(frame_id, stamp));
  ps.set__pose(nav::create_pose(x, y, z, qx, qy, qz, qw));
  return ps;
}

geometry_msgs::msg::PoseStamped nav::create_pose_stamped(const double& x, const double& y,
                                                         const double& z, const double& qx,
                                                         const double& qy, const double& qz,
                                                         const double& qw) {
  geometry_msgs::msg::PoseStamped ps;
  ps.header.set__frame_id("world");
  ps.set__pose(nav::create_pose(x, y, z, qx, qy, qz, qw));
  return ps;
}

// Euler angles RPY (extrinsic rotation, sequence "xyz") converted to normalized quaternion.
Eigen::Quaterniond nav::euler_to_quaternion(const double& roll, const double& pitch,
                                            const double& yaw) {
  Eigen::Quaterniond quat;
  quat = Eigen::AngleAxisd{yaw, Eigen::Vector3d::UnitZ()} *
         Eigen::AngleAxisd{pitch, Eigen::Vector3d::UnitY()} *
         Eigen::AngleAxisd{roll, Eigen::Vector3d::UnitX()};
  quat.normalize();

  return quat;
}

// Quaternion coefficients converted to Euler angles RPY. Quaternion normalized prior to conversion.
std::array<float, 3> nav::quaternion_to_euler(const double& x, const double& y, const double& z,
                                              const double& w) {
  Eigen::Quaterniond q{w, x, y, z};
  q.normalize();

  std::array<float, 3> angles{};

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  angles.at(0) = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = std::sqrt(1 + 2 * (q.w() * q.y() - q.x() * q.z()));
  double cosp = std::sqrt(1 - 2 * (q.w() * q.y() - q.x() * q.z()));
  angles.at(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  angles.at(2) = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

at_messages::msg::DynamicsState nav::dynamics_state_nan(
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp) {
  at_messages::msg::DynamicsState state;
  state.set__header(nav::create_header(frame_id, stamp));
  state.set__position(
      std::array<float, 6>{nanf(""), nanf(""), nanf(""), nanf(""), nanf(""), nanf("")});
  state.set__world_velocity(
      std::array<float, 6>{nanf(""), nanf(""), nanf(""), nanf(""), nanf(""), nanf("")});
  state.set__velocity(
      std::array<float, 6>{nanf(""), nanf(""), nanf(""), nanf(""), nanf(""), nanf("")});
  return state;
}

bool nav::equals(const geometry_msgs::msg::Pose& left, const geometry_msgs::msg::Pose& right) {
  bool x = false, y = false, z = false, angle = false;

  x = std::abs(left.position.x - right.position.x) <= 0.000001;
  y = std::abs(left.position.y - right.position.y) <= 0.000001;
  z = std::abs(left.position.z - right.position.z) <= 0.000001;

  Eigen::Quaterniond left_q{left.orientation.w, left.orientation.x, left.orientation.y,
                            left.orientation.z};
  Eigen::Quaterniond right_q{right.orientation.w, right.orientation.x, right.orientation.y,
                             right.orientation.z};
  Eigen::Quaterniond transition_q{left_q * right_q.inverse()};
  Eigen::AngleAxisd aa{transition_q};
  angle = std::abs(aa.angle() * (180 / M_PI)) <= 0.000001;

  return x && y && z && angle;
}
