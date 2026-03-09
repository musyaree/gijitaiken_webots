#include <rclcpp/rclcpp.hpp>
#include "gijitaiken_webots/node/vision_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gijitaiken_webots::VisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}