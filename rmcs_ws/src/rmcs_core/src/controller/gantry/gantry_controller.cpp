#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include <eigen3/Eigen/Core>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gantry {

class GantryController final : public rmcs_executor::Component, public rclcpp::Node {
    using Clock = std::chrono::steady_clock;
public:
    GantryController()
        : Node(get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        enabled_ = param<bool>("enabled", false);
        register_input("/remote/joystick/left", joystick_);
        register_input("/remote/switch/right", switch_);
        register_input("/gantry/feedback_valid", feedback_valid_);
        register_input("/gantry/remote_valid", remote_valid_);
        register_input("/predefined/update_rate", update_rate_);
        constexpr std::array names{"left_lift", "right_lift", "yaw"};
        for (size_t i = 0; i < names.size(); ++i) {
            const std::string path = std::string("/gantry/") + names[i];
            register_input(path + "/angle", angles_[i]);
            register_input(path + "/velocity", velocities_[i]);
            register_output(path + "/control_torque", torques_[i], 0.0);
            register_output(path + "/relative_angle", relative_[i], 0.0);
            register_output(path + "/target_velocity", target_velocities_[i], 0.0);
            const std::string prefix = i == 2 ? "yaw" : "lift";
            auto& pid = velocity_pid_[i];
            pid.kp = nonnegative(prefix + "_velocity_kp", 0.1);
            pid.ki = nonnegative(prefix + "_velocity_ki", 0.0);
            pid.kd = nonnegative(prefix + "_velocity_kd", 0.0);
            torque_limits_[i] = positive(prefix + "_torque_limit", 0.1);
            pid.output_min = -torque_limits_[i];
            pid.output_max = torque_limits_[i];
            pid.integral_max = positive(prefix + "_integral_limit", 1.0);
            pid.integral_min = -pid.integral_max;
        }
        lift_speed_ = positive("lift_target_speed", 0.03);
        yaw_speed_ = positive("yaw_target_speed", 0.03);
        lift_max_velocity_ = positive("lift_max_velocity", 0.05);
        yaw_max_velocity_ = positive("yaw_max_velocity", 0.05);
        lift_kp_ = positive("lift_position_kp", 2.0);
        yaw_kp_ = positive("yaw_position_kp", 2.0);
        sync_kp_ = nonnegative("sync_kp", 1.0);
        sync_limit_ = positive("sync_error_limit", 0.05);
        period_limit_ = positive("control_period_limit", 0.05);
        deadzone_ = nonnegative("joystick_deadzone", 0.05);
        if (deadzone_ >= 1.0) throw std::invalid_argument("gantry: deadzone must be below 1");
        pitch_sign_ = sign("pitch_input_sign", 1.0);
        yaw_sign_ = sign("yaw_input_sign", 1.0);
        lift_min_ = finite("lift_min_angle", -0.15);
        lift_max_ = finite("lift_max_angle", 0.15);
        yaw_min_ = finite("yaw_min_angle", -0.15);
        yaw_max_ = finite("yaw_max_angle", 0.15);
        if (lift_min_ >= 0 || lift_max_ <= 0 || yaw_min_ >= 0 || yaw_max_ <= 0)
            throw std::invalid_argument("gantry: relative ranges must contain zero");
        register_output("/gantry/lift_target", lift_target_, 0.0);
        register_output("/gantry/yaw_target", yaw_target_, 0.0);
        register_output("/gantry/sync_error", sync_error_, 0.0);
        register_output("/gantry/state", state_, 0.0);
        register_output("/gantry/fault", fault_, 0.0);
        register_output("/gantry/control_period", control_period_, 0.0);
    }

    void before_updating() override {
        RCLCPP_INFO(get_logger(),
            "Gantry enabled=%s. Manually level lift before initial DOWN/center calibration. "
            "Right DOWN=stop/arm, MIDDLE=stop, UP=control. Left stick x=lift, y=yaw.",
            enabled_ ? "true" : "false");
    }

    void update() override {
        const auto now = Clock::now();
        const auto dt = ticked_ ? std::chrono::duration<double>(now - last_tick_).count()
                               : 1.0 / *update_rate_;
        last_tick_ = now;
        ticked_ = true;
        *control_period_ = dt;
        if (!enabled_) { armed_ = false; stop(); return; }
        if (!std::isfinite(dt) || dt <= 0 || dt > period_limit_) { fail(3); return; }
        if (!*feedback_valid_) { fail(1); return; }
        if (!*remote_valid_) { fail(2); return; }
        if (!joystick_->allFinite() || joystick_->cwiseAbs().maxCoeff() > 1.01) {
            fail(4); return;
        }
        for (size_t i = 0; i < angles_.size(); ++i)
            if (!std::isfinite(*angles_[i]) || !std::isfinite(*velocities_[i])) {
                fail(4); return;
            }
        const bool centered = joystick_->cwiseAbs().maxCoeff() <= deadzone_;
        const auto sw = *switch_;
        // This is a manually confirmed relative reference, NOT automatic homing.
        // Keep these origins across faults/re-arming; restarting requires re-leveling.
        if (!referenced_) {
            stop();
            if (sw != rmcs_msgs::Switch::DOWN || !centered) return;
            for (size_t i = 0; i < angles_.size(); ++i) origins_[i] = *angles_[i];
            referenced_ = true;
            RCLCPP_INFO(get_logger(), "Gantry relative reference captured; this assumes lift is level.");
        }
        for (size_t i = 0; i < angles_.size(); ++i)
            *relative_[i] = *angles_[i] - origins_[i];
        *sync_error_ = *relative_[0] - *relative_[1];
        if (std::abs(*sync_error_) > sync_limit_) { fail(5); return; }
        if (*relative_[0] < lift_min_ || *relative_[0] > lift_max_
            || *relative_[1] < lift_min_ || *relative_[1] > lift_max_
            || *relative_[2] < yaw_min_ || *relative_[2] > yaw_max_) {
            fail(6); return;
        }
        if (sw == rmcs_msgs::Switch::DOWN) {
            if (centered) *fault_ = 0.0;
            stop();
            armed_ = centered;
            return;
        }
        if (sw != rmcs_msgs::Switch::UP) {
            armed_ = false;
            stop();
            return;
        }
        if (!armed_) { stop(); return; }
        if (!active_) {
            // No jump when activating: hold measured position, then accept motion.
            if (!centered) { stop(); return; }
            *lift_target_ = (*relative_[0] + *relative_[1]) / 2.0;
            *yaw_target_ = *relative_[2];
            active_ = true;
        }
        *lift_target_ = std::clamp(
            *lift_target_ + pitch_sign_ * axis(joystick_->x()) * lift_speed_ * dt,
            lift_min_, lift_max_);
        *yaw_target_ = std::clamp(
            *yaw_target_ + yaw_sign_ * axis(joystick_->y()) * yaw_speed_ * dt,
            yaw_min_, yaw_max_);
        for (size_t i = 0; i < angles_.size(); ++i) {
            const bool lift = i < 2;
            const double target = lift ? *lift_target_ : *yaw_target_;
            const double maximum = lift ? lift_max_velocity_ : yaw_max_velocity_;
            double velocity = (lift ? lift_kp_ : yaw_kp_) * (target - *relative_[i]);
            // If left is ahead, slow left and accelerate right by equal corrections.
            if (lift) velocity += (i == 0 ? -1 : 1) * sync_kp_ * *sync_error_;
            *target_velocities_[i] = std::clamp(velocity, -maximum, maximum);
            *torques_[i] = velocity_pid_[i].update(*target_velocities_[i] - *velocities_[i]);
            if (!std::isfinite(*torques_[i])) { fail(4); return; }
        }
        *state_ = 1.0; // Position hold and motion both keep the cascaded loops running.
    }

private:
    template <typename T> T param(const std::string& name, T fallback) {
        if (!has_parameter(name)) declare_parameter<T>(name, fallback);
        return get_parameter(name).get_value<T>();
    }
    double finite(const std::string& name, double fallback) {
        const auto value = param<double>(name, fallback);
        if (!std::isfinite(value)) throw std::invalid_argument("gantry: invalid " + name);
        return value;
    }
    double nonnegative(const std::string& name, double fallback) {
        const auto value = finite(name, fallback);
        if (value < 0) throw std::invalid_argument("gantry: negative " + name);
        return value;
    }
    double positive(const std::string& name, double fallback) {
        const auto value = nonnegative(name, fallback);
        if (value == 0) throw std::invalid_argument("gantry: zero " + name);
        return value;
    }
    double sign(const std::string& name, double fallback) {
        const auto value = finite(name, fallback);
        if (std::abs(value) != 1) throw std::invalid_argument("gantry: input sign must be +/-1");
        return value;
    }
    double axis(double value) const {
        if (std::abs(value) <= deadzone_) return 0.0;
        return std::copysign(std::min(1.0, (std::abs(value) - deadzone_) / (1 - deadzone_)), value);
    }
    void stop() {
        active_ = false;
        *state_ = *fault_ == 0.0 ? 0.0 : 2.0;
        for (size_t i = 0; i < torques_.size(); ++i) {
            *torques_[i] = *target_velocities_[i] = 0.0;
            velocity_pid_[i].reset();
        }
    }
    void fail(int reason) {
        armed_ = false;
        stop();
        *state_ = 2.0;
        if (*fault_ != reason)
            RCLCPP_WARN(get_logger(), "Gantry fault %d: zero torque; DOWN/center required to re-arm.", reason);
        *fault_ = static_cast<double>(reason);
    }
    bool enabled_, ticked_ = false, referenced_ = false, armed_ = false, active_ = false;
    Clock::time_point last_tick_{};
    double lift_speed_, yaw_speed_, lift_max_velocity_, yaw_max_velocity_;
    double lift_kp_, yaw_kp_, sync_kp_, sync_limit_, period_limit_, deadzone_;
    double pitch_sign_, yaw_sign_, lift_min_, lift_max_, yaw_min_, yaw_max_;
    std::array<double, 3> origins_{}, torque_limits_{};
    std::array<pid::PidCalculator, 3> velocity_pid_;
    std::array<InputInterface<double>, 3> angles_, velocities_;
    std::array<OutputInterface<double>, 3> torques_, relative_, target_velocities_;
    InputInterface<Eigen::Vector2d> joystick_;
    InputInterface<rmcs_msgs::Switch> switch_;
    InputInterface<bool> feedback_valid_, remote_valid_;
    InputInterface<double> update_rate_;
    OutputInterface<double> lift_target_, yaw_target_, sync_error_, state_, fault_, control_period_;
};
} // namespace rmcs_core::controller::gantry

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gantry::GantryController, rmcs_executor::Component)
