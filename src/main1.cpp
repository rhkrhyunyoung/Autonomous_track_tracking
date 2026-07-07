#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <vector>
#include <string>
#include "VisionProcessor.hpp"
#include "RobotController.hpp"
#include "config.hpp"

class LaneFollowerNode : public rclcpp::Node {
public:
    LaneFollowerNode() : Node("lane_follower_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10, std::bind(&LaneFollowerNode::image_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, std::bind(&LaneFollowerNode::imu_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);

        vp_ = std::make_unique<VisionProcessor>();
        rc_ = std::make_unique<RobotController>();
        RCLCPP_INFO(this->get_logger(), "🚀 덩어리 추적 & 시각화 모드 시작");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        current_yaw_rate_ = msg->angular_velocity.z;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
            cv::Mat binary = vp_->getBinaryTrack(frame); 
            cv::Mat display_bev = vp_->getBEV(frame);

            // 1. 덩어리(Contour) 찾기
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double target_x = 320.0; 
            bool track_found = false;
            int largest_idx = -1; // 가장 큰 덩어리의 번호 저장용

            // 2. 가장 큰 덩어리의 번호(인덱스) 찾기
            if (!contours.empty()) {
                double max_area = 0;
                for (size_t i = 0; i < contours.size(); i++) {
                    double area = cv::contourArea(contours[i]);
                    if (area > max_area) {
                        max_area = area;
                        largest_idx = i;
                    }
                }

                // 면적이 300 이상일 때만 트랙으로 인정
                if (largest_idx != -1 && max_area > 300) {
                    cv::Moments m = cv::moments(contours[largest_idx]);
                    if (m.m00 > 0) {
                        target_x = m.m10 / m.m00;
                        track_found = true;
                    }
                }
            }

            // 3. 트랙 놓쳤을 때 리셋 (Z값 고정 방지)
            if (!track_found) {
                rc_->reset(); 
                target_x = 320.0;
            }

            // 4. 필터 (80% 이전값 유지)
            target_x = (prev_target_x * 0.8) + (target_x * 0.2);
            prev_target_x = target_x;

            // 5. 제어값 계산
            double error = 320.0 - target_x; 
            double omega = rc_->calculateOmega(error / 100.0);

            geometry_msgs::msg::Twist twist;
            twist.linear.x = track_found ? Config::BASE_SPEED : 0.0;
            
            double final_omega = omega + (omega - current_yaw_rate_) * 0.05;
            twist.angular.z = std::clamp(final_omega, -0.8, 0.8);
            cmd_pub_->publish(twist);

            // 6. 시각화 (인덱스를 사용하여 안전하게 그리기)
            if (track_found && largest_idx != -1) {
                // 파란색으로 덩어리 외곽선 그리기
                cv::drawContours(display_bev, contours, largest_idx, cv::Scalar(255, 0, 0), 2);
                
                // 초록색으로 목표점과 방향선 그리기
                cv::circle(display_bev, cv::Point((int)target_x, 400), 10, cv::Scalar(0, 255, 0), -1);
                cv::line(display_bev, cv::Point(320, 480), cv::Point((int)target_x, 400), cv::Scalar(0, 255, 0), 3);
                
                // 조향값 텍스트 표시
                std::string txt = "Steer Z: " + std::to_string(twist.angular.z).substr(0, 5);
                cv::putText(display_bev, txt, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
            } else {
                cv::putText(display_bev, "TRACK LOST", cv::Point(200, 240), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
            }

            cv::imshow("BEV Result", display_bev);
            cv::waitKey(1);

        } catch (std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    std::unique_ptr<VisionProcessor> vp_;
    std::unique_ptr<RobotController> rc_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    double current_yaw_rate_ = 0.0, prev_target_x = 320.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
