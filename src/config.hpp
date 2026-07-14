#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace Config {
    const int IMAGE_WIDTH = 640;
    const int IMAGE_HEIGHT = 480;
    const std::vector<cv::Point2f> BEV_SRC = {
        cv::Point2f(0, 480),    cv::Point2f(640, 480),
        cv::Point2f(120, 220),  cv::Point2f(520, 220)
    };

    const std::vector<cv::Point2f> BEV_DST = {
        cv::Point2f(0, 480),    cv::Point2f(640, 480),
        cv::Point2f(0, 0),      cv::Point2f(640, 0)
    };

    const double TRACK_WIDTH_PX = 234.0; 

    const double PID_P = 0.4; 
    const double BASE_SPEED = 0.3;
    const double TRACK_WIDTH = 0.5;
    const double PID_I = 0.01;
    const double PID_D = 0.1;
}

#endif
