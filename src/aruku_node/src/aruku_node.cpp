#include "aruku_node/aruku_node.hpp" 

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "jitsuyo/config.hpp"
#include "keisan/angle.hpp"
#include <chrono>
#include <atomic>
#include <filesystem>
#include <vector>
#include <map> 

#include "tachimawari_mini/joint_id.hpp" 

using namespace std::chrono_literals;

ArukuNode::ArukuNode(const rclcpp::NodeOptions & options)
: Node("aruku_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting Aruku Node initialization...");
  
  walking_manager_ = std::make_shared<aruku::mini::WalkingManager>();

  std::string package_path = ament_index_cpp::get_package_share_directory("aruku_node");
  
  nlohmann::json kinematic_params;
  std::string config_path = package_path + "/config/"; 
  config_file_path_ = config_path + "kinematic.json";
  
  RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file_path_.c_str());
  
  if (jitsuyo::load_config(config_path, "kinematic.json", kinematic_params)) {
    walking_manager_->set_config(kinematic_params);
    last_config_time_ = std::filesystem::last_write_time(config_file_path_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load kinematic.json!");
  }

  // Setup ROS2
  set_joints_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&ArukuNode::cmd_vel_callback, this, std::placeholders::_1));
    
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/unit", 10, std::bind(&ArukuNode::imu_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    8ms, std::bind(&ArukuNode::control_loop_callback, this));
    
  RCLCPP_INFO(this->get_logger(), "Aruku Node is running.");
}

void ArukuNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  bool is_moving = (msg->linear.x != 0.0 || msg->linear.y != 0.0 || msg->angular.z != 0.0);
  
  if (is_moving) {
    double target_a_deg = keisan::make_radian(msg->angular.z).degree();
    walking_manager_->run(msg->linear.x, msg->linear.y, target_a_deg);
  } else {
    walking_manager_->stop();
  }
}

void ArukuNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  walking_manager_->update_gyro(
    keisan::Vector<3>(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z));
}

void ArukuNode::check_for_config_updates()
{
  try {
    auto current_config_time = std::filesystem::last_write_time(config_file_path_);
    if (current_config_time != last_config_time_) {
      RCLCPP_WARN(this->get_logger(), "Detected config file change");
      nlohmann::json kinematic_params;
      std::string config_path = std::filesystem::path(config_file_path_).parent_path().string() + "/";
      if (jitsuyo::load_config(config_path, "kinematic.json", kinematic_params)) {
        walking_manager_->set_config(kinematic_params);
        RCLCPP_INFO(this->get_logger(), "Config reloaded successfully.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to reload kinematic.json!");
      }
      last_config_time_ = current_config_time;
    }
  } catch (const std::filesystem::filesystem_error& e) {
    RCLCPP_ERROR_ONCE(this->get_logger(), "Filesystem error checking config: %s", e.what());
  }
}

const std::map<uint8_t, std::string> joint_id_to_name_map = {
    {tachimawari::joint::JointId::RIGHT_HIP_YAW, "right_hip_yaw"},
    {tachimawari::joint::JointId::RIGHT_HIP_ROLL, "right_hip_roll"},
    {tachimawari::joint::JointId::RIGHT_HIP_PITCH, "right_hip_pitch"},
    {tachimawari::joint::JointId::RIGHT_KNEE, "right_knee"},
    {tachimawari::joint::JointId::RIGHT_ANKLE_PITCH, "right_ankle_pitch"},
    {tachimawari::joint::JointId::RIGHT_ANKLE_ROLL, "right_ankle_roll"},
    {tachimawari::joint::JointId::LEFT_HIP_YAW, "left_hip_yaw"},
    {tachimawari::joint::JointId::LEFT_HIP_ROLL, "left_hip_roll"},
    {tachimawari::joint::JointId::LEFT_HIP_PITCH, "left_hip_pitch"},
    {tachimawari::joint::JointId::LEFT_KNEE, "left_knee"},
    {tachimawari::joint::JointId::LEFT_ANKLE_PITCH, "left_ankle_pitch"},
    {tachimawari::joint::JointId::LEFT_ANKLE_ROLL, "left_ankle_roll"},
    {tachimawari::joint::JointId::RIGHT_SHOULDER_PITCH, "right_shoulder_pitch"},
    {tachimawari::joint::JointId::RIGHT_SHOULDER_ROLL, "right_shoulder_roll"},
    {tachimawari::joint::JointId::RIGHT_ELBOW, "right_elbow"},
    {tachimawari::joint::JointId::LEFT_SHOULDER_PITCH, "left_shoulder_pitch"},
    {tachimawari::joint::JointId::LEFT_SHOULDER_ROLL, "left_shoulder_roll"},
    {tachimawari::joint::JointId::LEFT_ELBOW, "left_elbow"},
    {tachimawari::joint::JointId::NECK_YAW, "head_yaw"},
    {tachimawari::joint::JointId::NECK_PITCH, "head_pitch"}
};


void ArukuNode::control_loop_callback()
{
    check_for_config_updates();

    if (walking_manager_->process()) {
      const auto & joints = walking_manager_->get_joints(); 

      // Pesan JointState
      auto joints_msg = sensor_msgs::msg::JointState();
      joints_msg.header.stamp = this->get_clock()->now();

      for (const auto & joint : joints) {
        auto it = joint_id_to_name_map.find(joint.get_id());
        if (it != joint_id_to_name_map.end()) {
          joints_msg.name.push_back(it->second); // Nama sendi
          joints_msg.position.push_back(joint.get_position().radian()); // Posisi
        }
      }
      
      set_joints_pub_->publish(joints_msg);
    }
}