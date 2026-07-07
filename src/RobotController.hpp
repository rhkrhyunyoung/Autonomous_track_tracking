#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include "config.hpp"

class RobotController {
public:
    double calculateOmega(double error) {
        double p_term = Config::PID_P * error;
        error_sum_ += error;
        double i_term = Config::PID_I * error_sum_;
        double d_term = Config::PID_D * (error - prev_error_);
        prev_error_ = error;

        return p_term + i_term + d_term;
    }

    std::pair<double, double> getWheelSpeeds(double omega) {
        double left_v = Config::BASE_SPEED - (omega * Config::TRACK_WIDTH / 2.0);
        double right_v = Config::BASE_SPEED + (omega * Config::TRACK_WIDTH / 2.0);
        return {left_v, right_v};
    }

private:
    double error_sum_ = 0;
    double prev_error_ = 0;
};

#endif
