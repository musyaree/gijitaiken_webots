#include "aruku_node/aruku_node.hpp"
#include <memory>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<ArukuNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}