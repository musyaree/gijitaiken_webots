#ifndef GIJITAIKEN_WEBOTS__PROCESS__IMAGE_PROCESS_HPP_
#define GIJITAIKEN_WEBOTS__PROCESS__IMAGE_PROCESS_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace gijitaiken_webots {

struct ColorConfig {
    std::vector<int> min_hsv = {0, 0, 0};
    std::vector<int> max_hsv = {179, 255, 255};
    std::vector<int> min_lab = {0, -128, -128};
    std::vector<int> max_lab = {255, 127, 127};
    bool invert_hue = false;
    bool use_lab = false;
};

struct CameraConfig {
    int brightness = 128;
    int contrast = 128;
    int saturation = 128;
    int temperature = 4000;
    int exposure = 100;
    int gain = 50;
};

struct CamOffset {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
};

class ImageProcess {
public:
    static cv::Mat apply_offset_effects(const cv::Mat& in_image, const CamOffset& offset);
    static cv::Mat apply_camera_effects(const cv::Mat& in_image, const CameraConfig& config);
    static cv::Mat process_color_mask(const cv::Mat& in_image, const ColorConfig& config);
};

} // namespace gijitaiken_webots

#endif // GIJITAIKEN_WEBOTS__PROCESS__IMAGE_PROCESS_HPP_