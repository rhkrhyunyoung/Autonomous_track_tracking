#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <memory>
#include "VisionProcessor.hpp"
#include "RobotController.hpp"
#include "config.hpp"

class LaneFollowerNode : public rclcpp::Node {
public:
    LaneFollowerNode() : Node("lane_follower_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&LaneFollowerNode::image_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);

        vp_ = std::make_unique<VisionProcessor>();
        rc_ = std::make_unique<RobotController>();
        RCLCPP_INFO(this->get_logger(), "🚀 NUC 1280x720 모드 가동! (덩어리 추종)");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        cv::Mat binary = vp_->getBinaryTrack(frame);

        // 1. ROI 설정 (1280x720 해상도 기준 하단부)
        cv::Rect roi_rect(0, Config::ROI_START_Y, Config::IMAGE_WIDTH, Config::IMAGE_HEIGHT - Config::ROI_START_Y);
        cv::Mat roi_mask = binary(roi_rect);

        // 2. 덩어리 찾기
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roi_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double target_x = 640.0; // 1280의 정중앙
        bool track_found = false;

        if (!contours.empty()) {
            auto largest = *std::max_element(contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            // 1280 해상도에서는 덩어리 면적 기준을 1000 정도로 높였습니다.
            if (cv::contourArea(largest) > 1000) {
                cv::Moments m = cv::moments(largest);
                target_x = m.m10 / m.m00;
                track_found = true;

                // 시각화: 파란색 테두리
                std::vector<std::vector<cv::Point>> disp_cnts;
                std::vector<cv::Point> offset_cnt;
                for(auto& p : largest) offset_cnt.push_back(p + cv::Point(0, Config::ROI_START_Y));
                disp_cnts.push_back(offset_cnt);
                cv::drawContours(frame, disp_cnts, -1, cv::Scalar(255, 0, 0), 4);

                // 시각화: 초록색 중심점 및 가이드 라인
                cv::Point center_pt((int)target_x, Config::ROI_START_Y + (int)(m.m01/m.m00));
                cv::circle(frame, center_pt, 15, cv::Scalar(0, 255, 0), -1);
                cv::line(frame, cv::Point(640, 720), center_pt, cv::Scalar(0, 255, 0), 5);
            }
        }

        // 3. 제어 계산 (중앙 640 기준)
        double error = 640.0 - target_x; 
        double omega = rc_->calculateOmega(error);

        geometry_msgs::msg::Twist twist;
        if (track_found) {
            twist.linear.x = Config::BASE_SPEED;
            twist.angular.z = std::clamp(omega, -1.2, 1.2);
        } else {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
        cmd_pub_->publish(twist);

        // 너무 크면 보기 힘드니 절반으로 줄여서 출력
        cv::Mat display_frame;
        cv::resize(frame, display_frame, cv::Size(640, 360));
        cv::imshow("1280x720 Tracking View", display_frame);
        cv::waitKey(1);
    }

    std::unique_ptr<VisionProcessor> vp_;
    std::unique_ptr<RobotController> rc_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
