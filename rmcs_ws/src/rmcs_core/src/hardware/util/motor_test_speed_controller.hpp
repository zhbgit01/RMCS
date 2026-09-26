#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rmcs_core::hardware::util {

// Speed-mode PI only. Smooth the reference, not the corrective torque: a slow
// actuator slew limiter can keep accelerating a motor that is already too fast.
class MotorTestSpeedController {
public:
    struct Config {
        double kp = 0.008;
        double ki = 0.04; // torque / (rad/s * second), independent of update frequency
        double integral_limit = 0.08; // torque contribution, not accumulated error
        double output_limit = 0.08;
        double acceleration_limit = 6.0; // rad/s^2
    };

    void configure(Config config) {
        const auto nonnegative = [](double x) { return std::isfinite(x) && x >= 0; };
        if (!nonnegative(config.kp) || !nonnegative(config.ki)
            || !nonnegative(config.integral_limit) || !nonnegative(config.output_limit)
            || !nonnegative(config.acceleration_limit) || config.output_limit == 0
            || config.acceleration_limit == 0 || config.integral_limit > config.output_limit)
            throw std::invalid_argument("motor_test: invalid speed PI configuration");
        config_ = config;
        reset();
    }

    void reset() {
        reference_ = integral_ = proportional_ = unsaturated_ = last_request_ = 0.0;
    }

    double update(double requested_velocity, double measured_velocity, double dt) {
        if (!std::isfinite(requested_velocity) || !std::isfinite(measured_velocity)
            || !std::isfinite(dt) || dt <= 0)
            throw std::invalid_argument("motor_test: invalid speed PI input");
        // Centered speed stick releases torque immediately, including saved I.
        if (requested_velocity == 0.0) {
            reset();
            return 0.0;
        }
        if (last_request_ * requested_velocity < 0.0)
            integral_ = 0.0;
        last_request_ = requested_velocity;
        const double previous_reference = reference_;
        const double step = config_.acceleration_limit * dt;
        reference_ += std::clamp(requested_velocity - reference_, -step, step);
        if ((previous_reference > 0.0 && reference_ <= 0.0)
            || (previous_reference < 0.0 && reference_ >= 0.0))
            integral_ = 0.0;

        const double error = reference_ - measured_velocity;
        proportional_ = config_.kp * error;
        const double candidate = std::clamp(
            integral_ + config_.ki * error * dt,
            -config_.integral_limit, config_.integral_limit);
        const double candidate_output = proportional_ + candidate;
        // Conditional integration: never wind up further into output saturation,
        // but allow the integral to unwind as soon as the error reverses.
        if ((candidate_output <= config_.output_limit || candidate < integral_)
            && (candidate_output >= -config_.output_limit || candidate > integral_))
            integral_ = candidate;
        unsaturated_ = proportional_ + integral_;
        return std::clamp(unsaturated_, -config_.output_limit, config_.output_limit);
    }

    double reference() const { return reference_; }
    double proportional() const { return proportional_; }
    double integral() const { return integral_; }
    double unsaturated() const { return unsaturated_; }

private:
    Config config_{};
    double reference_ = 0.0, integral_ = 0.0, proportional_ = 0.0;
    double unsaturated_ = 0.0, last_request_ = 0.0;
};

} // namespace rmcs_core::hardware::util
