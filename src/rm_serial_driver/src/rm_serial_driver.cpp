// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include "rm_serial_driver/rm_serial_driver.hpp"

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include<iomanip>
#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <cmath>


namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{

  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();

  // Create Publisher
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(rclcpp::KeepLast(1)));
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
  velocity_pub_ = this->create_publisher<auto_aim_interfaces::msg::Velocity>("/current_velocity", 10);

  // Detect parameter client
  detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "armor_detector");

  // Tracker reset service client
  reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");

  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // Create Subscription
  send_sub_ = this->create_subscription<auto_aim_interfaces::msg::Send>(
    "/tracker/send", rclcpp::SensorDataQoS(),
    std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  // target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
  //   "/tracker/target", rclcpp::SensorDataQoS(),
  //   std::bind(&RMSerialDriver::sendData, this, std::placeholders::_1));

  //订阅cmd_vel目标信息
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&RMSerialDriver::cmdVelCallback, this, std::placeholders::_1));
}



RMSerialDriver::~RMSerialDriver()
{

}

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);

        serial_driver_->port()->receive(data);

        int detect_color=0;
        data.insert(data.begin(), header[0]);
        // for(int i=0;i<(int)data.size();i++)
        //   std::cout<<static_cast<int>(data[i])<<' ';
        // std::cout<<std::endl;
        ReceivePacket packet = fromVector(data);
        // std::cout<<packet.checksum;
        std::cout<<std::endl;
        //  std::cout<<"checksum"<<packet.checksum<<std::endl;
        // std::cout<<"send pitch:"<<packet.eulr.yaw<<std::endl;
        // std::cout<<sizeof(packet)<<std::endl;
        bool crc_ok =true;

        if (crc_ok) {

          packet.pitch=RMSerialDriver::pitch_re_trans(packet.eulr.pit);
          packet.yaw=RMSerialDriver::pitch_re_trans(packet.eulr.yaw);

          //std::cout<<packet.pitch<<std::endl;
          if (!initial_set_param_ || detect_color != previous_receive_color_) {
            setParam(rclcpp::Parameter("detect_color", detect_color));
            previous_receive_color_ = detect_color;
          }

          if (false) {
            resetTracker();
          }

          // 打印 data 结构体中的 xyz 和 yaw 值
          // std::cout << "xyz: (" << packet.aim_x << ", " << packet.aim_y << ", " << packet.aim_z << ")" << std::endl;
          // std::cout << "pitch: " << packet.pitch << "yaw: " << packet.yaw << std::endl;
          // RCLCPP_INFO(get_logger(), "CRC OK!");

          // //LOG [Receive] aim_xyz

          // RCLCPP_INFO(get_logger(), "[Receive] aim_x %f!", packet.aim_x);
          // RCLCPP_INFO(get_logger(), "[Receive] aim_y %f!", packet.aim_y);
          // RCLCPP_INFO(get_logger(), "[Receive] aim_z %f!", packet.aim_z);

          // // //LOG [Receive] [Receive] rpy
          // RCLCPP_INFO(get_logger(), "[Receive] roll %f!", packet.roll);
          // RCLCPP_INFO(get_logger(), "[Receive] pitch %f!", packet.pitch);
          //RCLCPP_INFO(get_logger(), "[Receive] yaw %f!", packet.yaw);

          sensor_msgs::msg::JointState joint_state;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          joint_state.header.stamp =
            this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          joint_state.name.push_back("pitch_joint");
          joint_state.name.push_back("yaw_joint");

          //float temp_pitch = pitch_re_trans(packet.pitch);
          joint_state.position.push_back(packet.pitch);

          //float temp_yaw = yaw_re_trans(packet.yaw);
          joint_state.position.push_back(packet.yaw);
          joint_state_pub_->publish(joint_state);

          auto_aim_interfaces::msg::Velocity current_velocity;
          timestamp_offset_ = this->get_parameter("timestamp_offset").as_double();
          current_velocity.header.stamp =
            this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
          current_velocity.velocity = packet.current_v;
          velocity_pub_->publish(current_velocity);

          if (abs(packet.aim_x) > 0.01) {
            aiming_point_.header.stamp = this->now();
            aiming_point_.pose.position.x = packet.aim_x;
            aiming_point_.pose.position.y = packet.aim_y;
            aiming_point_.pose.position.z = packet.aim_z;
            marker_pub_->publish(aiming_point_);
          }
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::sendData(const auto_aim_interfaces::msg::Send::SharedPtr msg)
{
  const static std::map<std::string, uint8_t> id_unit8_map{
    {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
    {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

  try {
    SendPacket packet;
    //packet.is_fire = msg->is_fire;
    // packet.x = msg->position.x;
    // packet.y = msg->position.y;
    // packet.z = msg->position.z;
    //packet.v_yaw = msg->v_yaw;
    //std::cout<<packet.pitch<<std::endl;
    std::cout<<"------------------------------"<<std::endl;
    packet.pitch = RMSerialDriver::pitch_trans(msg->pitch);
    packet.yaw = RMSerialDriver::pitch_trans(msg->yaw);
    packet.gimbal.pit=packet.pitch;
    packet.gimbal.yaw=packet.yaw;
    packet.gimbal.rol=0;
    packet.checksum=crc16::CRC16_Calc(reinterpret_cast<uint8_t *>(&packet), sizeof(packet), UINT16_MAX);

    // 打印 data 结构体中的 xyz 和 yaw 值
    // std::cout << "xyz: (" << packet.x << ", " << packet.y << ", " << packet.z << ")" << std::endl;
    // std::cout << "pitch: " << packet.pitch << "yaw: " << packet.yaw << std::endl;
    // RCLCPP_INFO(get_logger(), "[Send] aim_x %f!", packet.x);
    // RCLCPP_INFO(get_logger(), "[Send] aim_y %f!", packet.y);
    // RCLCPP_INFO(get_logger(), "[Send] aim_z %f!", packet.z);

    // RCLCPP_INFO(get_logger(), "-------------------------------------------------------------");
    // RCLCPP_INFO(get_logger(), "[Send] pitch %f!", packet.pitch);
    // RCLCPP_INFO(get_logger(), "[Send] yaw %f!", packet.yaw);
    // RCLCPP_INFO(get_logger(), "-------------------------------------------------------------");


    std::vector<uint8_t> data = toVector(packet);
    for(int i=0;i<(int)data.size();i++)
      std::cout<<static_cast<int>(data[i])<<" ";
    std::cout<<std::endl;
    serial_driver_->port()->send(data);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}

/*哨兵底盘*/
void RMSerialDriver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    SendPacket packet;

    // 限制速度，
    const double max_linear_speed = 1.0;  // 最大线速度 m/s
    const double max_angular_speed = 3.14; // 最大角速度 rad/s

    packet.vx = std::clamp(static_cast<float>(msg->linear.x),
                          -static_cast<float>(max_linear_speed),
                          static_cast<float>(max_linear_speed));
    packet.vy = std::clamp(static_cast<float>(msg->linear.y),
                          -static_cast<float>(max_linear_speed),
                          static_cast<float>(max_linear_speed));
    packet.wz = std::clamp(static_cast<float>(msg->angular.z),
                          -static_cast<float>(max_angular_speed),
                          static_cast<float>(max_angular_speed));

    packet.checksum = crc16::CRC16_Calc(reinterpret_cast<uint8_t *>(&packet),
                                       sizeof(packet) - sizeof(uint16_t),
                                       UINT16_MAX);

    RCLCPP_INFO(get_logger(), "Send velocity command - vx: %.2f, vy: %.2f, yaw_rate: %.2f",
                packet.vx, packet.vy, packet.wz);

    std::cout << "Velocity - vx: " << packet.vx
              << " vy: " << packet.vy
              << " wz: " << packet.wz << std::endl;

    std::vector<uint8_t> data(reinterpret_cast<uint8_t *>(&packet),
                             reinterpret_cast<uint8_t *>(&packet) + sizeof(packet));
    serial_driver_->port()->send(data);

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending velocity command: %s", ex.what());
    reopenPort();
  }
}
/**/

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::setParam(const rclcpp::Parameter & param)
{
  if (!detector_param_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping parameter set");
    return;
  }

  if (
    !set_param_future_.valid() ||
    set_param_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    RCLCPP_INFO(get_logger(), "Setting detect_color to %ld...", param.as_int());
    set_param_future_ = detector_param_client_->set_parameters(
      {param}, [this, param](const ResultFuturePtr & results) {
        for (const auto & result : results.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "Successfully set detect_color to %ld!", param.as_int());
        initial_set_param_ = true;
      });
  }
}

void RMSerialDriver::resetTracker()
{
  if (!reset_tracker_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
    return;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  reset_tracker_client_->async_send_request(request);
  RCLCPP_INFO(get_logger(), "Reset tracker!");
}

float RMSerialDriver::pitch_trans(float originAngle){

    if (originAngle < 0) {
      originAngle = originAngle +2* M_PI;
    }
    else 
      originAngle = originAngle;
    return originAngle;
    // return originAngle;
}


float RMSerialDriver::pitch_re_trans(float originAngle){

  if (originAngle <= M_PI) {
    originAngle = originAngle;
  }
  else
    originAngle = originAngle -2*M_PI;


      return originAngle;
  // return originAngle-M_PI;
}
float RMSerialDriver::yaw_trans(float originAngle){

    if (originAngle <=0) {
      originAngle = abs(originAngle);
    }
    else 
      originAngle = 2 * M_PI - originAngle;
    return originAngle;
}
float RMSerialDriver::yaw_re_trans(float originAngle){

    if (originAngle <= M_PI) {
      originAngle = -originAngle;
    }
    else 
      originAngle = 2*M_PI  -originAngle;
    return originAngle;


}


}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
