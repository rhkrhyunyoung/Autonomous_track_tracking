#ifndef VISION_PROCESSOR_HPP
#define VISION_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include "config.hpp"

class VisionProcessor {
public:
    cv::Mat getBinaryTrack(const cv::Mat& frame) {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // 민트색/초록색 트랙 범위 (상황에 따라 조금씩 조절)
        cv::inRange(hsv, cv::Scalar(35, 40, 40), cv::Scalar(90, 255, 255), mask);

        // 노이즈 제거 (침식 후 팽창)
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        return mask;
    }
};
#endif
