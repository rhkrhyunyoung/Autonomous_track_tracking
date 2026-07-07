#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP
#include "config.hpp"

class RobotController {
public:
    double calculateOmega(double error) {
        // P 제어만 사용 (가장 안정적임)
        double p_term = Config::PID_P * error;
        
        // D 제어 (급격한 변화 방지)
        double d_term = Config::PID_D * (error - prev_error_);
        prev_error_ = error;

        return p_term + d_term;
    }

    // 트랙 놓쳤을 때 에러 기록 삭제용
    void reset() {
        prev_error_ = 0;
    }

private:
    double prev_error_ = 0;
};
#endif
