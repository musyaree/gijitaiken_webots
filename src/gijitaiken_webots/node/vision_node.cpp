#include "gijitaiken_webots/node/vision_node.hpp"
#include <sstream>

namespace gijitaiken_webots {

VisionNode::VisionNode() : Node("vision_controller") {
    this->active_object = "ball";
    ColorConfig default_config;
    this->color_config_map["ball"] = default_config;
    this->color_config_map["field"] = default_config;

    this->image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image", 10,
        std::bind(&VisionNode::image_callback, this, std::placeholders::_1));

    this->processed_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/processed_image", 10);

    // 3. SERVICES
    using namespace std::placeholders;
    this->get_color_service = create_service<ninshiki_interfaces::srv::GetColorSetting>(
        "ninshiki_cpp/config/get_color_setting", std::bind(&VisionNode::get_color_callback, this, _1, _2));
    this->set_color_service = create_service<ninshiki_interfaces::srv::SetColorSetting>(
        "ninshiki_cpp/config/set_color_setting", std::bind(&VisionNode::set_color_callback, this, _1, _2));
    this->save_color_service = create_service<ninshiki_interfaces::srv::SaveColorSetting>(
        "ninshiki_cpp/config/save_color_setting", std::bind(&VisionNode::save_color_callback, this, _1, _2));

    this->get_camera_service = create_service<shisen_interfaces::srv::GetCaptureSetting>(
        "shisen_cpp/config/get_capture_setting", std::bind(&VisionNode::get_camera_callback, this, _1, _2));
    this->update_camera_service = create_service<shisen_interfaces::srv::UpdateCaptureSetting>(
        "shisen_cpp/config/update_capture_setting", std::bind(&VisionNode::update_camera_callback, this, _1, _2));

    this->get_offset_service = create_service<gyakuenki_interfaces::srv::GetCameraOffset>(
        "/camera/get_camera_offset", std::bind(&VisionNode::get_offset_callback, this, _1, _2));
    this->update_offset_service = create_service<gyakuenki_interfaces::srv::UpdateCameraOffset>(
        "/camera/update_camera_offset", std::bind(&VisionNode::update_offset_callback, this, _1, _2));

    std::cout << "[VISION] C++ Node Ready" << std::endl;
}

void VisionNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        cv::Mat offset_img = ImageProcess::apply_offset_effects(cv_ptr->image, this->cam_offset);
        cv::Mat effect_img = ImageProcess::apply_camera_effects(offset_img, this->camera_config);

        if (color_config_map.find(active_object) != color_config_map.end()) {
            ColorConfig current_color_cfg = color_config_map[active_object];
            cv::Mat result = ImageProcess::process_color_mask(effect_img, current_color_cfg);
            
            sensor_msgs::msg::Image::SharedPtr out_msg = cv_bridge::CvImage(
                msg->header, sensor_msgs::image_encodings::BGR8, result).toImageMsg();
            this->processed_image_publisher->publish(*out_msg);
        }

    } catch (cv_bridge::Exception& e) {
        printf("cv_bridge exception: %s", e.what());
    }
}


void VisionNode::get_offset_callback(const std::shared_ptr<gyakuenki_interfaces::srv::GetCameraOffset::Request>,
                                     std::shared_ptr<gyakuenki_interfaces::srv::GetCameraOffset::Response> res) {
    res->position_x = this->cam_offset.x;
    res->position_y = this->cam_offset.y;
    res->position_z = this->cam_offset.z;
    res->roll = this->cam_offset.roll;
    res->pitch = this->cam_offset.pitch;
    res->yaw = this->cam_offset.yaw;
    res->status = true;
}

void VisionNode::update_offset_callback(const std::shared_ptr<gyakuenki_interfaces::srv::UpdateCameraOffset::Request> req,
                                        std::shared_ptr<gyakuenki_interfaces::srv::UpdateCameraOffset::Response> res) {
    this->cam_offset.x = req->position_x;
    this->cam_offset.y = req->position_y;
    this->cam_offset.z = req->position_z;
    this->cam_offset.roll = req->roll;
    this->cam_offset.pitch = req->pitch;
    this->cam_offset.yaw = req->yaw;
    res->status = true;
}

void VisionNode::get_camera_callback(const std::shared_ptr<shisen_interfaces::srv::GetCaptureSetting::Request>,
                                     std::shared_ptr<shisen_interfaces::srv::GetCaptureSetting::Response> res) {
    res->brightness = this->camera_config.brightness;
    res->contrast = this->camera_config.contrast;
    res->saturation = this->camera_config.saturation;
    res->temperature = this->camera_config.temperature;
    res->exposure = this->camera_config.exposure;
    res->gain = this->camera_config.gain;
}

void VisionNode::update_camera_callback(const std::shared_ptr<shisen_interfaces::srv::UpdateCaptureSetting::Request> req,
                                        std::shared_ptr<shisen_interfaces::srv::UpdateCaptureSetting::Response> res) {
    this->camera_config.brightness = req->brightness;
    this->camera_config.contrast = req->contrast;
    this->camera_config.saturation = req->saturation;
    this->camera_config.temperature = req->temperature;
    this->camera_config.exposure = req->exposure;
    this->camera_config.gain = req->gain;
    res->success = true;
}

void VisionNode::set_color_callback(const std::shared_ptr<ninshiki_interfaces::srv::SetColorSetting::Request> req,
                                    std::shared_ptr<ninshiki_interfaces::srv::SetColorSetting::Response> res) {
    std::string obj = req->name;
    ColorConfig cfg;
    
    cfg.min_hsv = {req->min_hue / 2, req->min_saturation, req->min_value};
    cfg.max_hsv = {req->max_hue / 2, req->max_saturation, req->max_value};
    
    cfg.invert_hue = req->invert_hue;

    cfg.min_lab = {0, -128, -128};
    cfg.max_lab = {255, 127, 127};
    cfg.use_lab = false;

    this->color_config_map[obj] = cfg;
    this->active_object = obj;
    res->success = true;
}

void VisionNode::save_color_callback(const std::shared_ptr<ninshiki_interfaces::srv::SaveColorSetting::Request>,
                                     std::shared_ptr<ninshiki_interfaces::srv::SaveColorSetting::Response> res) {
    // TODO(musyaree): implement save color callback
    res->success = true;
}

void VisionNode::get_color_callback(const std::shared_ptr<ninshiki_interfaces::srv::GetColorSetting::Request>,
                                    std::shared_ptr<ninshiki_interfaces::srv::GetColorSetting::Response> res) {
    res->json_color = generate_json_string();
}

std::string VisionNode::generate_json_string() {
    std::stringstream ss;
    ss << "{";
    auto it = this->color_config_map.begin();
    while (it != this->color_config_map.end()) {
        ss << "\"" << it->first << "\": {";
        ss << "\"min_hsv\": [" << it->second.min_hsv[0] << "," << it->second.min_hsv[1] << "," << it->second.min_hsv[2] << "],";
        ss << "\"max_hsv\": [" << it->second.max_hsv[0] << "," << it->second.max_hsv[1] << "," << it->second.max_hsv[2] << "],";
        ss << "\"min_lab\": [" << it->second.min_lab[0] << "," << it->second.min_lab[1] << "," << it->second.min_lab[2] << "],";
        ss << "\"max_lab\": [" << it->second.max_lab[0] << "," << it->second.max_lab[1] << "," << it->second.max_lab[2] << "],";
        ss << "\"invert_hue\": " << (it->second.invert_hue ? "true" : "false") << ",";
        ss << "\"use_lab\": " << (it->second.use_lab ? "true" : "false");
        ss << "}";
        if (++it != this->color_config_map.end()) ss << ",";
    }
    ss << "}";
    return ss.str();
}

} // namespace gijitaiken_webots