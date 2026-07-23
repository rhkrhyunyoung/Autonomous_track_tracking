#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace Config {
    const int IMAGE_WIDTH = 640;
    const int IMAGE_HEIGHT = 480;

    /*
     * BEV_SRC: 원본 영상에서 가져올 영역 (사다리꼴)
     * 너무 멀리까지 보지 않고, 로봇 앞바닥을 적당히 사다리꼴로 잡습니다.
     */
    const std::vector<cv::Point2f> BEV_SRC = {
        cv::Point2f(600.0f, 460.0f),  // 오른쪽 아래
        cv::Point2f(40.0f, 460.0f),   // 왼쪽 아래
        cv::Point2f(220.0f, 300.0f),  // 왼쪽 위
        cv::Point2f(420.0f, 300.0f)   // 오른쪽 위
    };

    /*
     * BEV_DST: 결과 영상에서 뿌려질 영역 (직사각형)
     * 위쪽을 832까지 벌렸던 걸 440으로 확 줄여서 억지로 늘어나는 왜곡을 없앴습니다.
     * 이제 트랙이 11자로 곧게 보일 거예요.
     */
    const std::vector<cv::Point2f> BEV_DST = {
        cv::Point2f(440.0f, 480.0f),  // 오른쪽 아래
        cv::Point2f(200.0f, 480.0f),  // 왼쪽 아래
        cv::Point2f(200.0f, 0.0f),    // 왼쪽 위
        cv::Point2f(440.0f, 0.0f)     // 오른쪽 위
    };

    // PD 제어 파라미터 (적당한 값)
    const double PID_P = 0.4; 
    const double PID_D = 0.1;
    const double BASE_SPEED = 0.3;
}
#endif
