#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "VisionProcessor.hpp"
#include "RobotController.hpp"
#include "config.hpp"

class LaneFollowerNode : public rclcpp::Node {
public:
    LaneFollowerNode() : Node("lane_follower_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", 10,
            std::bind(&LaneFollowerNode::image_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_nav", 10);

        vp_ = std::make_unique<VisionProcessor>();
        rc_ = std::make_unique<RobotController>();

        current_imu_x_ = 0.0;
        imu_received_ = false;

        RCLCPP_INFO(this->get_logger(), "🚀 IMU 방향 수정 완료: -3(우회전) / 3(좌회전)");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        current_imu_x_ = msg->linear_acceleration.x;
        imu_received_ = true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        cv::Mat binary = vp_->getBinaryTrack(frame);

        int img_w = frame.cols;
        int img_h = frame.rows;
        double center_x = img_w / 2.0;

        int start_y = Config::ROI_START_Y;
        int end_y   = Config::ROI_END_Y;
        start_y = std::clamp(start_y, 0, img_h);
        end_y   = std::clamp(end_y, start_y, img_h);

        cv::Rect roi_rect(0, start_y, img_w, end_y - start_y);
        cv::Mat roi_mask = binary(roi_rect);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(roi_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double target_x = center_x;
        double visual_cx = center_x; 
        double visual_cy = (end_y - start_y) / 2.0;
        bool track_found = false;
        bool is_tracking = false;

        if (!contours.empty()) {
            auto largest = *std::max_element(contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            double area = cv::contourArea(largest);
            if (area > 1000.0) {
                track_found = true;
                cv::Moments m = cv::moments(largest);
                if (m.m00 != 0.0) {
                    visual_cx = m.m10 / m.m00;
                    visual_cy = m.m01 / m.m00;
                    if (area >= 50000.0) { target_x = visual_cx; is_tracking = true; }
                    else { target_x = center_x; is_tracking = false; }
                    
                    std::vector<std::vector<cv::Point>> disp_cnts;
                    std::vector<cv::Point> offset_cnt;
                    for (const auto& p : largest) offset_cnt.push_back(p + cv::Point(0, start_y));
                    disp_cnts.push_back(offset_cnt);
                    cv::drawContours(frame, disp_cnts, -1, cv::Scalar(255, 0, 0), 4);
                    
                    // 초록색 점: 원래 비전이 찾은 차선의 정중앙
                    cv::circle(frame, cv::Point(static_cast<int>(visual_cx), start_y + static_cast<int>(visual_cy)), 15, cv::Scalar(0, 255, 0), -1);
                }
            }
        }
        
        geometry_msgs::msg::Twist twist;
        if (track_found) {
            if (is_tracking) {
                double speed_factor = std::max(0.6, 1.0 - std::abs(final_omega) * 0.5);
                twist.linear.x = Config::BASE_SPEED * speed_factor;
                twist.angular.z = std::clamp(final_omega, -Config::MAX_ANGULAR_SPEED, Config::MAX_ANGULAR_SPEED);
            } else {
                twist.linear.x = Config::BASE_SPEED;
                twist.angular.z = 0.0;
            }
        } else {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }
        cmd_pub_->publish(twist);

        cv::rectangle(frame, roi_rect, cv::Scalar(0, 255, 255), 2);
        cv::Mat display_frame;
        cv::resize(frame, display_frame, cv::Size(640, 360));
        cv::imshow("Lane Tracking View", display_frame);
        cv::waitKey(1);
    }

    std::unique_ptr<VisionProcessor> vp_;
    std::unique_ptr<RobotController> rc_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    double current_imu_x_;
    bool imu_received_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
