#include "aruku_mini/aruku_mujoco_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "jitsuyo/config.hpp"
#include "keisan/angle.hpp"
#include <chrono>
#include <atomic>

ArukuMujocoNode::ArukuMujocoNode(const rclcpp::NodeOptions & options) : Node("aruku_mujoco_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting Aruku Mujoco Node initialization...");

  std::string package_path = ament_index_cpp::get_package_share_directory("aruku_mujoco");
  std::string model_path = package_path + "/model/scene.xml";

  simulator_ = std::make_shared<Simulator>(model_path);
  control_manager_ = std::make_shared<tachimawari::control::MujocoControlManager>(simulator_);
  walking_manager_ = std::make_shared<aruku::mini::WalkingManager>(control_manager_);

  nlohmann::json kinematic_params;
  std::string config_path = package_path + "/config/";
  if (jitsuyo::load_config(config_path, "kinematic.json", kinematic_params)) {
    walking_manager_->set_config(kinematic_params);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load kinematic.json!");
  }

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(8), std::bind(&ArukuMujocoNode::control_loop_callback, this));
    
  RCLCPP_INFO(this->get_logger(), "Aruku Mujoco Node is running. Use arrow keys to move.");
}

std::shared_ptr<Simulator> ArukuMujocoNode::get_simulator()
{
  return simulator_;
}

void ArukuMujocoNode::handle_keyboard(int key, int act)
{
  const double speed = 30.0;
  const double turn_speed_deg = 20.0;

  if (act == GLFW_PRESS) {
      switch (key) {
            case GLFW_KEY_W:
            case GLFW_KEY_UP:
                RCLCPP_INFO(this->get_logger(), "Command: Walk Forward");
                walking_manager_->run(speed, 0.0, 0.0);
                break;
            case GLFW_KEY_S:
            case GLFW_KEY_DOWN:
                RCLCPP_INFO(this->get_logger(), "Command: Walk Backward");
                walking_manager_->run(-speed, 0.0, 0.0);
                break;
            case GLFW_KEY_LEFT:
                RCLCPP_INFO(this->get_logger(), "Command: Turn Left");
                walking_manager_->run(0.0, 0.0, -turn_speed_deg);
                break;
            case GLFW_KEY_RIGHT:
                RCLCPP_INFO(this->get_logger(), "Command: Turn Right");
                walking_manager_->run(0.0, 0.0, turn_speed_deg);
                break;
            case GLFW_KEY_D:
                RCLCPP_INFO(this->get_logger(), "Command: Walk Right");
                walking_manager_->run(0.0, speed, 0.0);
                break;
            case GLFW_KEY_A:
                RCLCPP_INFO(this->get_logger(), "Command: Walk Left");
                walking_manager_->run(0.0, -speed, 0.0);
                break;
            case GLFW_KEY_R:
                request_reset_();
                break;
      }
  } else if (act == GLFW_RELEASE) {
      if (key == GLFW_KEY_UP || key == GLFW_KEY_W || 
          key == GLFW_KEY_DOWN || key == GLFW_KEY_S || 
          key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT || 
          key == GLFW_KEY_D || key == GLFW_KEY_A) {
          RCLCPP_INFO(this->get_logger(), "Command: Stop");
          walking_manager_->stop();
      }
  }
}

void ArukuMujocoNode::request_reset_() 
{
    reset_requested_ = true;
}

void ArukuMujocoNode::control_loop_callback()
{
    if (reset_requested_) {
        RCLCPP_INFO(this->get_logger(), "Executing reset...");
        walking_manager_->stop();
        simulator_->reset();
        reset_requested_ = false;
        return;
    }

    if (simulator_ && simulator_->is_window_open()) {
        auto gyro_data = simulator_->get_gyro();
        if (gyro_data.size() == 3) {
        walking_manager_->update_gyro(keisan::Vector<3>(gyro_data[0], gyro_data[1], gyro_data[2]));
        }
        walking_manager_->process();
        simulator_->step();
    } else if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}