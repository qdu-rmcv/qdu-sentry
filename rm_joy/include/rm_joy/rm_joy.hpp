#ifndef RM_JOY__RM_JOY_HPP_
#define RM_JOY__RM_JOY_HPP_

#include <map>
#include <memory>
#include <string>

#include "example_interfaces/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rm_joy {

class TeleopTwistJoyNode : public rclcpp::Node {
public:
  explicit TeleopTwistJoyNode(const rclcpp::NodeOptions &options);

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                     const std::string &which_map);
  void fillCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                     const std::string &which_map,
                     geometry_msgs::msg::Twist *cmd_vel_msg);

  double getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                const std::map<std::string, int64_t> &axis_map,
                const std::map<std::string, double> &scale_map,
                const std::string &fieldname);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      cmd_vel_stamped_pub_;

  bool publish_stamped_twist_;
  std::string robot_base_frame_;
  bool require_enable_button_;
  int64_t enable_button_;
  int64_t enable_turbo_button_;
  bool inverted_reverse_;

  std::map<std::string, int64_t> axis_chassis_map_;
  std::map<std::string, std::map<std::string, double>> scale_chassis_map_;

  bool sent_disable_msg_;
  double dt_;
};

} // namespace rm_joy

#endif // RM_JOY__RM_JOY_HPP_
