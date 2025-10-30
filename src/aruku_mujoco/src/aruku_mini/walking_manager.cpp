#include "aruku_mini/walking_manager.hpp"
#include "tachimawari_mini/joint_id.hpp"
#include <iostream>
#include "rclcpp/rclcpp.hpp"

namespace aruku::mini
{

WalkingManager::WalkingManager(std::shared_ptr<tachimawari::control::ControlManager> control_manager)
: control_manager(control_manager)
{
  // Inisialisasi vektor sendi
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

bool WalkingManager::process()
{
  // 1. Jalankan satu siklus perhitungan kinematik
  if (kinematic.run_kinematic()) {
    // 2. Dapatkan hasil array sudut dari kinematik
    const auto & angles = kinematic.get_angles();

    // 3. Masukkan hasil sudut ke dalam vektor sendi
    for (auto & joint : joints) {
      // Indeks di array `angles` sama dengan ID sendi
      joint.set_position(angles[joint.get_id()]);
    }

    // 4. Kirim seluruh perintah sendi ke jembatan MuJoCo
    if (control_manager) {
      control_manager->sync_write_packet(joints);
    }
    return true;
  }
  if (is_running()) { 
        RCLCPP_WARN(rclcpp::get_logger("walking_manager"), "Kinematic process failed! Check parameters in kinematic.json");
  }

  return false;
}

bool WalkingManager::is_running() const
{
  return kinematic.get_running_state();
}

void WalkingManager::update_gyro(const keisan::Vector<3> & gyro)
{
  kinematic.update_gyro(gyro);
}

}  // namespace aruku::mini