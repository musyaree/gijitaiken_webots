#ifndef GIJITAIKEN_WEBOTS__NODE__VISION_NODE_HPP_
#define GIJITAIKEN_WEBOTS__NODE__VISION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <map>
#include <string>

#include "ninshiki_interfaces/srv/get_color_setting.hpp"
#include "ninshiki_interfaces/srv/set_color_setting.hpp"
#include "ninshiki_interfaces/srv/save_color_setting.hpp"
#include "shisen_interfaces/srv/get_capture_setting.hpp"
#include "shisen_interfaces/srv/update_capture_setting.hpp"
#include "gyakuenki_interfaces/srv/get_camera_offset.hpp"
#include "gyakuenki_interfaces/srv/update_camera_offset.hpp"

#include "gijitaiken_webots/process/image_process.hpp"

namespace gijitaiken_webots {

class VisionNode : public rclcpp::Node {
public:
    VisionNode();

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    void get_color_callback(const std::shared_ptr<ninshiki_interfaces::srv::GetColorSetting::Request> req,
                      std::shared_ptr<ninshiki_interfaces::srv::GetColorSetting::Response> res);
    void set_color_callback(const std::shared_ptr<ninshiki_interfaces::srv::SetColorSetting::Request> req,
                      std::shared_ptr<ninshiki_interfaces::srv::SetColorSetting::Response> res);
    void save_color_callback(const std::shared_ptr<ninshiki_interfaces::srv::SaveColorSetting::Request> req,
                       std::shared_ptr<ninshiki_interfaces::srv::SaveColorSetting::Response> res);

    void get_camera_callback(const std::shared_ptr<shisen_interfaces::srv::GetCaptureSetting::Request> req,
                            std::shared_ptr<shisen_interfaces::srv::GetCaptureSetting::Response> res);
    void update_camera_callback(const std::shared_ptr<shisen_interfaces::srv::UpdateCaptureSetting::Request> req,
                       std::shared_ptr<shisen_interfaces::srv::UpdateCaptureSetting::Response> res);

    void get_offset_callback(const std::shared_ptr<gyakuenki_interfaces::srv::GetCameraOffset::Request> req,
                       std::shared_ptr<gyakuenki_interfaces::srv::GetCameraOffset::Response> res);
    void update_offset_callback(const std::shared_ptr<gyakuenki_interfaces::srv::UpdateCameraOffset::Request> req,
                          std::shared_ptr<gyakuenki_interfaces::srv::UpdateCameraOffset::Response> res);

    std::string generate_json_string();

    std::string active_object;
    std::map<std::string, ColorConfig> color_config_map;
    CameraConfig camera_config;
    CamOffset cam_offset;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_publisher;

    rclcpp::Service<ninshiki_interfaces::srv::GetColorSetting>::SharedPtr get_color_service;
    rclcpp::Service<ninshiki_interfaces::srv::SetColorSetting>::SharedPtr set_color_service;
    rclcpp::Service<ninshiki_interfaces::srv::SaveColorSetting>::SharedPtr save_color_service;
    
    rclcpp::Service<shisen_interfaces::srv::GetCaptureSetting>::SharedPtr get_camera_service;
    rclcpp::Service<shisen_interfaces::srv::UpdateCaptureSetting>::SharedPtr update_camera_service;
    
    rclcpp::Service<gyakuenki_interfaces::srv::GetCameraOffset>::SharedPtr get_offset_service;
    rclcpp::Service<gyakuenki_interfaces::srv::UpdateCameraOffset>::SharedPtr update_offset_service;
};

} // namespace gijitaiken_webots

#endif // GIJITAIKEN_WEBOTS__NODE__VISION_NODE_HPP_