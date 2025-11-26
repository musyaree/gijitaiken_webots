#ifndef ARUKU_NODE__ARUKU_NODE_HPP_
#define ARUKU_NODE__ARUKU_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "tachimawari_mini/joint.hpp" 
#include "aruku_mini/walking_manager.hpp"

#include <memory>
#include <atomic>
#include <filesystem>

class ArukuNode : public rclcpp::Node
{
public:
  explicit ArukuNode(const rclcpp::NodeOptions & options);

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  void control_loop_callback();
  void check_for_config_updates();

  std::shared_ptr<aruku::mini::WalkingManager> walking_manager_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::string last_log_msg_ = "";
  std::string config_file_path_;
  std::filesystem::file_time_type last_config_time_;
  
  // Setup ROS2
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr set_joints_pub_; 
};

#endif // ARUKU_NODE__ARUKU_NODE_HPP_