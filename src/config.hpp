#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace Config {

    // 카메라 해상도
    const int IMAGE_WIDTH = 848;
    const int IMAGE_HEIGHT = 480;

    // 주행 제어
    const double PID_P = 0.0035;
    const double PID_D = 0.0008;
    const double MAX_ANGULAR_SPEED = 1.8;
    const double BASE_SPEED = 0.25;

    // ROI 설정
    // 상단 40% 제거
    const int ROI_START_Y = static_cast<int>(IMAGE_HEIGHT * 0.2);

    // 하단 10% 제거
    // 즉 화면 높이의 90% 위치까지만 사용
    const int ROI_END_Y = static_cast<int>(IMAGE_HEIGHT * 0.9);

}

#endif
