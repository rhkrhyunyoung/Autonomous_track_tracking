#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP
#include "config.hpp"

class RobotController {
public:
    double calculateOmega(double error) {
        double p_term = Config::PID_P * error;
        double d_term = Config::PID_D * (error - prev_error_);
        prev_error_ = error;
        return p_term + d_term;
    }

    void reset() {
        prev_error_ = 0;
    }

private:
    double prev_error_ = 0;
};
#endif
