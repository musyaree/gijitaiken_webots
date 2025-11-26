#include "aruku_mini/walking_manager.hpp"
#include "tachimawari_mini/joint_id.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

namespace aruku::mini
{

WalkingManager::WalkingManager()
{
  for (const auto& [name, id] : tachimawari::joint::JointId::by_name) {
    joints.emplace_back(id);
  }
}

bool WalkingManager::set_config(const nlohmann::json & kinematic_data)
{
  try {
    kinematic.set_config(kinematic_data);
    return true;
  } catch (const std::exception & e) {
    std::cerr << "Error setting kinematic config: " << e.what() << std::endl;
    return false;
  }
}

void WalkingManager::run(double x_move, double y_move, double a_move)
{
  if (!is_running()) {
    kinematic.set_running_state(true);
  }
  
  kinematic.set_move_amplitude(x_move, y_move, keisan::make_degree(a_move), false);
}

void WalkingManager::stop()
{
  kinematic.set_running_state(false);
}

void WalkingManager::update_gyro(const keisan::Vector<3> & gyro)
{
  kinematic.update_gyro(gyro);
}

bool WalkingManager::process()
{
  bool success = kinematic.run_kinematic();

  const auto & angles = kinematic.get_angles();

  for (auto & joint : joints) {
    joint.set_position(angles[joint.get_id()]);
  }
    
  if (!success && is_running()) { 
        RCLCPP_WARN(rclcpp::get_logger("walking_manager"), "Kinematic process failed!");
  }

  return true;
}

bool WalkingManager::is_running() const
{
  return kinematic.get_running_state();
}

const std::vector<tachimawari::joint::Joint> & WalkingManager::get_joints() const
{
  return joints;
}

}  // namespace aruku::mini