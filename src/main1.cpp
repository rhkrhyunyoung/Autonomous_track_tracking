#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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
            "/camera/camera/color/image_raw",
            10,
            std::bind(
                &LaneFollowerNode::image_callback,
                this,
                std::placeholders::_1
            )
        );

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel_nav",
            10
        );

        vp_ = std::make_unique<VisionProcessor>();
        rc_ = std::make_unique<RobotController>();

        RCLCPP_INFO(
            this->get_logger(),
            "🚀 라인 트레이서 (상단 40%% / 하단 10%% 제외 ROI)"
        );
    }

private:
    void image_callback(
        const sensor_msgs::msg::Image::SharedPtr msg
    ) {
        // ============================================================
        // 1. ROS Image -> OpenCV
        // ============================================================
        cv::Mat frame =
            cv_bridge::toCvShare(msg, "bgr8")->image.clone();

        cv::Mat binary = vp_->getBinaryTrack(frame);

        int img_w = frame.cols;
        int img_h = frame.rows;

        double center_x = img_w / 2.0;


        // ============================================================
        // 2. ROI 설정
        //
        // ROI 값은 config.hpp에서만 관리
        //
        // 848x480 기준:
        // start_y = 192  -> 상단 40% 제거
        // end_y   = 432  -> 하단 10% 제거
        //
        // 실제 사용 영역:
        // y = 192 ~ 431
        // ============================================================

        int start_y = Config::ROI_START_Y;
        int end_y   = Config::ROI_END_Y;

        // 혹시 잘못된 설정으로 OpenCV assertion이 발생하는 것을 방지
        start_y = std::clamp(start_y, 0, img_h);
        end_y   = std::clamp(end_y, start_y, img_h);

        cv::Rect roi_rect(
            0,
            start_y,
            img_w,
            end_y - start_y
        );

        cv::Mat roi_mask = binary(roi_rect);


        // ============================================================
        // 3. ROI 내부에서 contour 찾기
        // ============================================================

        std::vector<std::vector<cv::Point>> contours;

        cv::findContours(
            roi_mask,
            contours,
            cv::RETR_EXTERNAL,
            cv::CHAIN_APPROX_SIMPLE
        );


        double target_x = center_x;

        bool track_found = false;
        bool is_tracking = false;


        // ============================================================
        // 4. 가장 큰 contour 선택
        // ============================================================

        if (!contours.empty()) {

            auto largest = *std::max_element(
                contours.begin(),
                contours.end(),
                [](const std::vector<cv::Point>& a,
                   const std::vector<cv::Point>& b) {

                    return cv::contourArea(a)
                         < cv::contourArea(b);
                }
            );

            double area = cv::contourArea(largest);


            // 너무 작은 노이즈 제거
            if (area > 1000.0) {

                track_found = true;

                cv::Moments m = cv::moments(largest);

                // m00 = 0 보호
                if (m.m00 != 0.0) {

                    double cx = m.m10 / m.m00;
                    double cy = m.m01 / m.m00;


                    // ====================================================
                    // 면적이 50000 이상이면 실제 조향
                    // 그보다 작으면 직진
                    // ====================================================

                    if (area >= 50000.0) {

                        target_x = cx;
                        is_tracking = true;

                    } else {

                        target_x = center_x;
                        is_tracking = false;
                    }


                    // ====================================================
                    // 5. 시각화
                    // ====================================================

                    // ROI 좌표계의 contour를
                    // 원본 영상 좌표계로 변환
                    std::vector<std::vector<cv::Point>> disp_cnts;
                    std::vector<cv::Point> offset_cnt;

                    for (const auto& p : largest) {

                        offset_cnt.push_back(
                            p + cv::Point(0, start_y)
                        );
                    }

                    disp_cnts.push_back(offset_cnt);


                    // 파란색 contour
                    cv::drawContours(
                        frame,
                        disp_cnts,
                        -1,
                        cv::Scalar(255, 0, 0),
                        4
                    );


                    // contour 중심점
                    cv::Point center_pt(
                        static_cast<int>(cx),
                        start_y + static_cast<int>(cy)
                    );


                    // 초록 중심점
                    cv::circle(
                        frame,
                        center_pt,
                        15,
                        cv::Scalar(0, 255, 0),
                        -1
                    );


                    // 화면 아래 중앙 -> 검출 중심
                    cv::line(
                        frame,
                        cv::Point(
                            static_cast<int>(center_x),
                            img_h
                        ),
                        center_pt,
                        cv::Scalar(0, 255, 0),
                        5
                    );


                    // contour 면적 표시
                    cv::putText(
                        frame,
                        "Area: " + std::to_string(
                            static_cast<int>(area)
                        ),
                        cv::Point(30, 110),
                        cv::FONT_HERSHEY_SIMPLEX,
                        1.0,
                        cv::Scalar(255, 255, 255),
                        2
                    );
                }
            }
        }


        // ============================================================
        // 6. 제어값 계산
        // ============================================================

        double error = center_x - target_x;

        double omega =
            rc_->calculateOmega(error);


        geometry_msgs::msg::Twist twist;


        if (track_found) {

            // --------------------------------------------------------
            // 라인이 충분히 크게 검출됨
            // -> PID 조향
            // --------------------------------------------------------
            if (is_tracking) {

                double speed_factor =
                    std::max(
                        0.6,
                        1.0 - std::abs(omega) * 0.5
                    );

                twist.linear.x =
                    Config::BASE_SPEED * speed_factor;

                twist.angular.z =
                    std::clamp(
                        omega,
                        -Config::MAX_ANGULAR_SPEED,
                        Config::MAX_ANGULAR_SPEED
                    );
            }

            // --------------------------------------------------------
            // contour는 있지만 면적이 50000 미만
            // -> 강제 직진
            // --------------------------------------------------------
            else {

                twist.linear.x =
                    Config::BASE_SPEED;

                twist.angular.z = 0.0;
            }
        }

        // ------------------------------------------------------------
        // contour 자체가 없음
        // -> 정지
        // ------------------------------------------------------------
        else {

            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        }


        cmd_pub_->publish(twist);


        // ============================================================
        // 7. 화면 상태 표시
        // ============================================================

        std::string mode_str;

        if (is_tracking) {

            mode_str = "MODE: TRACKING";

        } else if (track_found) {

            mode_str = "MODE: FORCE STRAIGHT";

        } else {

            mode_str = "MODE: NO TRACK";
        }


        cv::putText(
            frame,
            mode_str,
            cv::Point(30, 60),
            cv::FONT_HERSHEY_SIMPLEX,
            1.2,
            is_tracking
                ? cv::Scalar(0, 255, 0)
                : cv::Scalar(0, 255, 255),
            3
        );


        // ============================================================
        // ROI 노란색 박스
        // ============================================================

        cv::rectangle(
            frame,
            roi_rect,
            cv::Scalar(0, 255, 255),
            2
        );


        // 화면 표시용 resize
        cv::Mat display_frame;

        cv::resize(
            frame,
            display_frame,
            cv::Size(640, 360)
        );

        cv::imshow(
            "Lane Tracking View",
            display_frame
        );

        cv::waitKey(1);
    }


    std::unique_ptr<VisionProcessor> vp_;
    std::unique_ptr<RobotController> rc_;

    rclcpp::Subscription<
        sensor_msgs::msg::Image
    >::SharedPtr image_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::Twist
    >::SharedPtr cmd_pub_;
};


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LaneFollowerNode>()
    );

    rclcpp::shutdown();

    return 0;
}
