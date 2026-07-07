#ifndef VISION_PROCESSOR_HPP
#define VISION_PROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "config.hpp"

class VisionProcessor {
public:
    VisionProcessor() {
        M_ = cv::getPerspectiveTransform(Config::BEV_SRC, Config::BEV_DST);
    }

    cv::Mat getBEV(const cv::Mat& frame) {
        cv::Mat bev;
        cv::warpPerspective(frame, bev, M_, cv::Size(Config::IMAGE_WIDTH, Config::IMAGE_HEIGHT));
        return bev;
    }

    cv::Mat getBinaryTrack(const cv::Mat& frame) {
        cv::Mat bev = getBEV(frame);
        cv::Mat hsv, mask;
        cv::cvtColor(bev, hsv, cv::COLOR_BGR2HSV);

        // [수정] 민트색 트랙 범위 (H: 60~100 사이)
        cv::Scalar lower_mint(40, 40, 40); 
        cv::Scalar upper_mint(110, 255, 255);
        cv::inRange(hsv, lower_mint, upper_mint, mask);

        // 노이즈 제거
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));
        
        // 화면 상단 30%는 노이즈 방지를 위해 자름
        mask(cv::Rect(0, 0, Config::IMAGE_WIDTH, (int)(Config::IMAGE_HEIGHT * 0.3))).setTo(0);
        return mask;
    }

    std::vector<double> fitPolynomial(const std::vector<cv::Point>& pts) {
        if (pts.size() < 20) return {};
        Eigen::MatrixXd A(pts.size(), 3);
        Eigen::VectorXd b(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            double y = pts[i].y;
            A(i, 0) = y * y; A(i, 1) = y; A(i, 2) = 1.0;
            b(i) = pts[i].x;
        }
        Eigen::Vector3d coeff = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        if (std::abs(coeff[0]) > 0.005) coeff[0] = 0; // 너무 꺾이면 직선 처리
        return {coeff[0], coeff[1], coeff[2]};
    }

private:
    cv::Mat M_;
};
#endif
