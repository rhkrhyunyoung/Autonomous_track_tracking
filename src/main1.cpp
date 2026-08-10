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
            "/camera_front/color/image_raw", 10, std::bind(&LaneFollowerNode::image_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);

        vp_ = std::make_unique<VisionProcessor>();
        rc_ = std::make_unique<RobotController>();
        RCLCPP_INFO(this->get_logger(), "🚀 라인 트레이서 (하단 60% ROI 모드)");
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        cv::Mat binary = vp_->getBinaryTrack(frame);

        int img_w = frame.cols;
        int img_h = frame.rows;
        double center_x = img_w / 2.0;

        // [핵심 수정] 하단 60%를 잡으려면 상단 40%를 버려야 함 (0.4 곱함)
        // 만약 Config 값을 강제로 쓰고 싶다면 start_y = Config::ROI_START_Y; 로 바꾸세요.
        int start_y = (int)(img_h * 0.4); 
        
        cv::Rect roi_rect(0, start_y, img_w, img_h - start_y);
        cv::Mat roi_mask = binary(roi_rect);

        // 2. 덩어리 찾기
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roi_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double target_x = center_x; 
        bool track_found = false;
        bool is_tracking = false;

        if (!contours.empty()) {
            auto largest = *std::max_element(contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            double area = cv::contourArea(largest);

            if (area > 1000) {
                track_found = true;
                cv::Moments m = cv::moments(largest);
                double cx = m.m10 / m.m00;
                double cy = m.m01 / m.m00;

                // 500,000 기준 조향 결정
                if (area >= 50000) {
                    target_x = cx;
                    is_tracking = true;
                } else {
                    target_x = center_x;
                }

                // --- 시각화 무조건 수행 ---
                // 1. 파란색 테두리 (offset을 start_y로 정확히 줌)
                std::vector<std::vector<cv::Point>> disp_cnts;
                std::vector<cv::Point> offset_cnt;
                for(auto& p : largest) offset_cnt.push_back(p + cv::Point(0, start_y));
                disp_cnts.push_back(offset_cnt);
                cv::drawContours(frame, disp_cnts, -1, cv::Scalar(255, 0, 0), 4);

                // 2. 초록색 중심점 및 라인 (화면 맨 아래부터 연결)
                cv::Point center_pt((int)cx, start_y + (int)cy);
                cv::circle(frame, center_pt, 15, cv::Scalar(0, 255, 0), -1);
                cv::line(frame, cv::Point((int)center_x, img_h), center_pt, cv::Scalar(0, 255, 0), 5);
                
                // 3. 면적 텍스트 표시
                cv::putText(frame, "Area: " + std::to_string((int)area), cv::Point(30, 110), 
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
            }
        }

        // 3. 제어 신호
        double error = center_x - target_x; 
        double omega = rc_->calculateOmega(error);

        geometry_msgs::msg::Twist twist;
        if (track_found) {
            if (is_tracking) {
                double speed_factor = std::max(0.6, 1.0 - std::abs(omega) * 0.5);
                twist.linear.x = Config::BASE_SPEED * speed_factor;
                twist.angular.z = std::clamp(omega, -Config::MAX_ANGULAR_SPEED, Config::MAX_ANGULAR_SPEED);
            } else {
                twist.linear.x = Config::BASE_SPEED; // 20만 미만일 땐 직진
                twist.angular.z = 0.0;
            }
        } else {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
        cmd_pub_->publish(twist);

        // 상태 표시 및 ROI 노란 박스
        std::string mode_str = is_tracking ? "MODE: TRACKING" : "MODE: FORCE STRAIGHT";
        cv::putText(frame, mode_str, cv::Point(30, 60), cv::FONT_HERSHEY_SIMPLEX, 1.2, 
                    is_tracking ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 255, 255), 3);
        
        cv::rectangle(frame, roi_rect, cv::Scalar(0, 255, 255), 2); // ROI 박스

        cv::Mat display_frame;
        cv::resize(frame, display_frame, cv::Size(640, 360));
        cv::imshow("Lane Tracking View", display_frame);
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
