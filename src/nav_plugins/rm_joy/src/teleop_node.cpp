#include "rm_joy/rm_joy.hpp"

#include <cinttypes>

namespace rm_joy {

TeleopTwistJoyNode::TeleopTwistJoyNode(const rclcpp::NodeOptions &options)
    : Node("teleop_twist_joy_node", options) {
  RCLCPP_INFO(this->get_logger(), "Starting Teleop Twist Joy");

  this->declare_parameter<bool>("publish_stamped_twist", false);
  this->declare_parameter<std::string>("robot_base_frame", "base_link");
  this->declare_parameter<bool>("require_enable_button", false);
  this->declare_parameter<int64_t>("enable_button", 5);
  this->declare_parameter<int64_t>("enable_turbo_button", -1);
  this->declare_parameter<bool>("inverted_reverse", false);

  this->declare_parameters<int64_t>("axis_chassis",
                                    {{"x", 1L}, {"y", 0L}, {"yaw", 3L}});
  this->declare_parameters<double>("scale_chassis",
                                   {{"x", 0.5}, {"y", 0.5}, {"yaw", 0.5}});
  this->declare_parameters<double>("scale_chassis_turbo",
                                   {{"x", 1.0}, {"y", 1.0}, {"yaw", 1.0}});

  this->get_parameter("publish_stamped_twist", publish_stamped_twist_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("require_enable_button", require_enable_button_);
  this->get_parameter("enable_button", enable_button_);
  this->get_parameter("enable_turbo_button", enable_turbo_button_);
  this->get_parameter("inverted_reverse", inverted_reverse_);
  this->get_parameters("axis_chassis", axis_chassis_map_);
  this->get_parameters("scale_chassis", scale_chassis_map_["normal"]);
  this->get_parameters("scale_chassis_turbo", scale_chassis_map_["turbo"]);

  // 订阅手柄输入
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&TeleopTwistJoyNode::joyCallback, this, std::placeholders::_1));

  if (publish_stamped_twist_) {
    cmd_vel_stamped_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
  } else {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }
}

void TeleopTwistJoyNode::fillCmdVelMsg(
  const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string & which_map,
  geometry_msgs::msg::Twist * cmd_vel_msg)
{
  double lin_x = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "x");
  double ang_z = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "yaw");

  cmd_vel_msg->linear.x = lin_x;
  cmd_vel_msg->linear.y = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "y");
  cmd_vel_msg->linear.z = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "z");
  cmd_vel_msg->angular.z = (lin_x < 0.0 && inverted_reverse_) ? -ang_z : ang_z;
  cmd_vel_msg->angular.y =
    getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "pitch");
  cmd_vel_msg->angular.x =
    getVal(joy_msg, axis_chassis_map_, scale_chassis_map_[which_map], "roll");
}

void TeleopTwistJoyNode::joyCallback(
    const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
  std::stringstream ss;
  ss << "Axes: ";
  for (size_t i = 0; i < joy_msg->axes.size(); ++i) {
    ss << joy_msg->axes[i] << " ";
  }
  ss << "Buttons: ";
  for (size_t i = 0; i < joy_msg->buttons.size(); ++i) {
    ss << joy_msg->buttons[i] << " ";
  }
  RCLCPP_INFO(this->get_logger(), "Received joy message: %s", ss.str().c_str());
  // 获取手柄左侧和右侧摇杆的输入
  double x = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_["normal"], "x");
  double y = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_["normal"], "y");
  double yaw = getVal(joy_msg, axis_chassis_map_, scale_chassis_map_["normal"], "yaw");
  RCLCPP_INFO(this->get_logger(), "Calculated values: x=%f, y=%f, yaw=%f", x, y, yaw);

  // 创建并发布速度消息
  auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
  fillCmdVelMsg(joy_msg, "normal", cmd_vel_msg.get());

  if (publish_stamped_twist_) {
    auto cmd_vel_stamped_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd_vel_stamped_msg->header.stamp = this->now();
    cmd_vel_stamped_msg->header.frame_id = robot_base_frame_;
    cmd_vel_stamped_msg->twist = *cmd_vel_msg;
    cmd_vel_stamped_pub_->publish(std::move(cmd_vel_stamped_msg));
    RCLCPP_INFO(this->get_logger(), "Published stamped cmd_vel message");
  } else {
    cmd_vel_pub_->publish(std::move(cmd_vel_msg));
    RCLCPP_INFO(this->get_logger(), "Published cmd_vel message");
  }

  // 调试输出，确认发布的数据
  RCLCPP_INFO(this->get_logger(),
              "Publishing: linear.x = %f, linear.y = %f, angular.z = %f", x, y,
              yaw);
}

double TeleopTwistJoyNode::getVal(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                           const std::map<std::string, int64_t> &axis_map,
                           const std::map<std::string, double> &scale_map,
                           const std::string &fieldname) {
  auto axis_it = axis_map.find(fieldname);
  auto scale_it = scale_map.find(fieldname);

  if (axis_it == axis_map.end() || axis_it->second == -1L ||
      scale_it == scale_map.end() ||
      static_cast<int>(joy_msg->axes.size()) <= axis_it->second) {
    return 0.0;
  }

  return joy_msg->axes[axis_it->second] * scale_it->second;
}

} // namespace rm_joy
