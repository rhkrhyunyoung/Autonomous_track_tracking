#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
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
        RCLCPP_INFO(this->get_logger(), "🚀 안정화 모드 가동! (부호 반전 및 뱅크 대응)");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        current_yaw_rate_ = msg->angular_velocity.z;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
       cv::Mat frame = cv_ptr->image;
      cv::Mat binary = vp_->getBinaryTrack(frame); // 민트색 이진화 영상
      cv::Mat display_bev = vp_->getBEV(frame);

    // [핵심] 선 찾기 포기! 가장 큰 민트색 덩어리(Contour) 찾기
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      double target_x = 320.0; // 기본값은 정중앙
      bool track_found = false;

      if (!contours.empty()) {
        // 1. 가장 면적이 큰 덩어리 찾기
         auto largest_contour = *std::max_element(contours.begin(), contours.end(),
             [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                  return cv::contourArea(a) < cv::contourArea(b);
            });

        // 2. 덩어리가 너무 작지 않으면 그 중심점을 타겟으로 잡음
        if (cv::contourArea(largest_contour) > 500) {
            cv::Moments m = cv::moments(largest_contour);
            target_x = m.m10 / m.m00;
            track_found = true;
        }
    }

    // 3. 필터 (움직임을 부드럽게)
    if (!track_found) target_x = prev_target_x; // 못 찾으면 가던 대로
    target_x = (prev_target_x * 0.8) + (target_x * 0.2);
    prev_target_x = target_x;

    // 4. 제어값 계산 (오른쪽이면 음수, 왼쪽이면 양수 - 방향 다시 확인됨)
    // target_x > 320 (오른쪽) -> error < 0 -> omega < 0 (우회전)
    double error = 320.0 - target_x; 
    double omega = rc_->calculateOmega(error / 100.0);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = Config::BASE_SPEED;
    twist.angular.z = std::clamp(omega + (omega - current_yaw_rate_) * 0.1, -1.0, 1.0);

    cmd_pub_->publish(twist);

    // [시각화] 이제 덩어리 외곽선과 중심점을 보여줍니다.
    if (track_found) {
        cv::drawContours(display_bev, contours, -1, cv::Scalar(255, 0, 0), 2); // 파란색 외곽선
        cv::circle(display_bev, cv::Point((int)target_x, 400), 10, cv::Scalar(0, 255, 0), -1); // 초록색 점
        cv::line(display_bev, cv::Point(320, 480), cv::Point((int)target_x, 400), cv::Scalar(0, 255, 0), 3);
    }

    cv::imshow("BEV Result", display_bev);
    cv::waitKey(1);
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
