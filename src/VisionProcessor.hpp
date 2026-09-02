#ifndef VISION_PROCESSOR_HPP
#define VISION_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include "config.hpp"

class VisionProcessor {
public:
    // 이름을 다시 getBinaryTrack으로 원복
    cv::Mat getBinaryTrack(const cv::Mat& frame) {
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // 1. 민트색과 초록색을 모두 아우르도록 색상(Hue) 및 채도(Saturation) 범위 넓히기
        cv::inRange(hsv, cv::Scalar(25, 25, 25), cv::Scalar(95, 255, 255), mask);

        // 2. 노이즈 제거 및 끊어진 트랙 연결 (모폴로지 연산 강화)
        cv::Mat kernel_small = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat kernel_large = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));

        // 작은 잡음 제거 (Open)
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_small);
        
        // 끊어진 트랙 영역을 메우기 위해 CLOSE 연산 사용
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_large);

        return mask;
    }
};
#endif
