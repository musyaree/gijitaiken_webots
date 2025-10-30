#include "aruku_mini/aruku_mujoco_node.hpp"
#include <thread>
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArukuMujocoNode>(rclcpp::NodeOptions());
  
  auto simulator = node->get_simulator();
  GLFWwindow* window = simulator->get_window();

  glfwSetWindowUserPointer(window, node.get());
  
  std::thread ros_thread([&]() {
    rclcpp::spin(node);
  });

  while (rclcpp::ok() && simulator && simulator->is_window_open()) {
    simulator->render();
  }

  rclcpp::shutdown();
  ros_thread.join();
  return 0;
}