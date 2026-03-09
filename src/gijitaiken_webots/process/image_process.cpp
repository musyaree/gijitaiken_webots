#include "gijitaiken_webots/process/image_process.hpp"

namespace gijitaiken_webots {

cv::Mat ImageProcess::apply_offset_effects(const cv::Mat& in_image, const CamOffset& offset) {
    if (in_image.empty()) return in_image;

    cv::Mat img = in_image.clone();
    int rows = img.rows;
    int cols = img.cols;

    if (offset.roll != 0.0) {
        cv::Point2f center(cols / 2.0f, rows / 2.0f);
        cv::Mat rot_mat = cv::getRotationMatrix2D(center, offset.roll, 1.0);
        cv::warpAffine(img, img, rot_mat, cv::Size(cols, rows));
    }

    double shift_x = offset.x + offset.yaw;
    double shift_y = offset.y + offset.pitch;

    if (shift_x != 0.0 || shift_y != 0.0) {
        cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, shift_x, 0, 1, shift_y);
        cv::warpAffine(img, img, trans_mat, cv::Size(cols, rows));
    }

    if (offset.z > 0.0) {
        int crop_amount = static_cast<int>(offset.z);
        if (crop_amount < rows / 2 && crop_amount < cols / 2) {
            cv::Rect roi(crop_amount, crop_amount, cols - 2 * crop_amount, rows - 2 * crop_amount);
            img = img(roi);
            cv::resize(img, img, cv::Size(cols, rows));
        }
    }

    return img;
}

cv::Mat ImageProcess::apply_camera_effects(const cv::Mat& in_image, const CameraConfig& config) {
    if (in_image.empty()) return in_image;
    cv::Mat img = in_image.clone();

    double alpha = config.contrast / 128.0;
    double beta = config.brightness - 128.0;
    img.convertTo(img, -1, alpha, beta);

    double sat_scale = config.saturation / 128.0;
    if (sat_scale != 1.0) {
        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> channels;
        cv::split(hsv, channels);
        channels[1].convertTo(channels[1], -1, sat_scale, 0);
        cv::merge(channels, hsv);
        cv::cvtColor(hsv, img, cv::COLOR_HSV2BGR);
    }
    return img;
}

cv::Mat ImageProcess::process_color_mask(const cv::Mat& in_image, const ColorConfig& config) {
    if (in_image.empty()) return in_image;

    cv::Mat src_for_mask, lower, upper;
    if (config.use_lab) {
        cv::cvtColor(in_image, src_for_mask, cv::COLOR_BGR2Lab);
        lower = (cv::Mat_<uint8_t>(1, 3) << config.min_lab[0], (uint8_t)config.min_lab[1], (uint8_t)config.min_lab[2]);
        upper = (cv::Mat_<uint8_t>(1, 3) << config.max_lab[0], (uint8_t)config.max_lab[1], (uint8_t)config.max_lab[2]);
    } else {
        cv::cvtColor(in_image, src_for_mask, cv::COLOR_BGR2HSV);
        lower = (cv::Mat_<uint8_t>(1, 3) << config.min_hsv[0], config.min_hsv[1], config.min_hsv[2]);
        upper = (cv::Mat_<uint8_t>(1, 3) << config.max_hsv[0], config.max_hsv[1], config.max_hsv[2]);
    }

    cv::Mat mask; 
    cv::inRange(src_for_mask, lower, upper, mask); 

    cv::Mat result;
    cv::bitwise_and(in_image, in_image, result, mask);
    return result;
}

} // namespace gijitaiken_webots