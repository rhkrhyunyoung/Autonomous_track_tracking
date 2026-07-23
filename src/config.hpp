#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace Config {
    const int IMAGE_WIDTH = 1280;
    const int IMAGE_HEIGHT = 720;

    // 화면이 2배 커졌으므로 에러값도 2배가 됩니다. 
    // 제어 안정성을 위해 PID_P를 기존의 절반 정도로 낮췄습니다.
    const double PID_P = 0.0015; 
    const double PID_D = 0.0005; 
    const double BASE_SPEED = 0.25; 

    // 720 높이 기준 하단 영역 (450~720 사이 탐색)
    const int ROI_START_Y = 450; 
}
#endif
