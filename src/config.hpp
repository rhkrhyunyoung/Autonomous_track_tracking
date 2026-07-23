#ifndef CONFIG_HPP
#define CONFIG_HPP

namespace Config {
    const int IMAGE_WIDTH = 1280;
    const int IMAGE_HEIGHT = 720;

    const double PID_P = 0.0035; 
    const double PID_D = 0.0008; 
    const double MAX_ANGULAR_SPEED = 1.8;
    const double BASE_SPEED = 0.25; 

    const int ROI_START_Y = 450; 
}
#endif
