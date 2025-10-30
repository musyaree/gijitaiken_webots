#ifndef ARUKU_MINI__ARUKU_MUJOCO_NODE_HPP_
#define ARUKU_MINI__ARUKU_MUJOCO_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "aruku_mini/walking_manager.hpp"
#include "tachimawari_mini/mujoco_control_manager.hpp"
#include "simulator.hpp"
#include <memory>
#include <atomic>

class ArukuMujocoNode : public rclcpp::Node
{
public:
  explicit ArukuMujocoNode(const rclcpp::NodeOptions & options);
  std::shared_ptr<Simulator> get_simulator();
  void handle_keyboard(int key, int act);
  void request_reset_();

private:
  void walking_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void control_loop_callback();

  std::shared_ptr<Simulator> simulator_;
  std::shared_ptr<tachimawari::control::MujocoControlManager> control_manager_;
  std::shared_ptr<aruku::mini::WalkingManager> walking_manager_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::atomic<bool> reset_requested_{false};
};

#endif // ARUKU_MINI__ARUKU_MUJOCO_NODE_HPP_