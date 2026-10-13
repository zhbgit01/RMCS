#include <chrono>
#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include "controller/gantry/gantry_logic.hpp"
#include "controller/pid/pid_calculator.hpp"

namespace rmcs_core::controller::gantry {
class GantryController : public rmcs_executor::Component, public rclcpp::Node {
public:
    GantryController() : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)),
        logic_(GantryLogic::Config{
            get_parameter("deadzone").as_double(), get_parameter("lift_rate").as_double(),
            get_parameter("yaw_rate").as_double(), get_parameter("position_kp").as_double(),
            get_parameter("sync_kp").as_double(), get_parameter("max_velocity").as_double(),
            get_parameter("max_skew").as_double()}) {
        register_input("/remote/joystick/left", joystick_);
        register_input("/remote/switch/right", switch_);
        register_input("/remote/valid", remote_valid_);
        register_input("/gantry/feedback_valid", feedback_valid_);
        register_input("/predefined/timestamp", timestamp_);
        pitch_sign_ = get_parameter("pitch_sign").as_double();
        yaw_sign_ = get_parameter("yaw_sign").as_double();
        if (std::abs(pitch_sign_) != 1 || std::abs(yaw_sign_) != 1) throw std::invalid_argument("stick signs must be +/-1");
        const double kp = get_parameter("velocity_kp").as_double();
        const double limit = get_parameter("torque_limit").as_double();
        if (!std::isfinite(kp) || kp < 0 || !std::isfinite(limit) || limit <= 0) throw std::invalid_argument("invalid velocity PID parameters");
        const std::array<std::string, 3> names{"yaw", "left", "right"};
        for (int i = 0; i < 3; ++i) {
            auto prefix = "/gantry/" + names[i];
            register_input(prefix + "/angle", positions_[i]);
            register_input(prefix + "/velocity", velocities_[i]);
            register_output(prefix + "/control_torque", torques_[i], 0.0);
            register_output(prefix + "/target", targets_[i], 0.0);
            pid_[i].kp = kp; pid_[i].ki = pid_[i].kd = 0;
            pid_[i].output_min = -limit; pid_[i].output_max = limit;
        }
        register_output("/gantry/active", active_, false);
        register_output("/gantry/fault", fault_, false);
        register_output("/gantry/skew", skew_, 0.0);
    }
    void update() override {
        GantryLogic::Input input;
        auto now = *timestamp_;
        input.dt = initialized_ ? std::chrono::duration<double>(now - previous_).count() : 0.001;
        previous_ = now; initialized_ = true;
        input.healthy = *remote_valid_ && *feedback_valid_ && joystick_->allFinite()
            && (*switch_ == rmcs_msgs::Switch::UP || *switch_ == rmcs_msgs::Switch::MIDDLE || *switch_ == rmcs_msgs::Switch::DOWN);
        input.enable = *switch_ == rmcs_msgs::Switch::UP;
        // DR16 uses x for channel3 (vertical), negative y for channel2 (horizontal).
        input.pitch_stick = pitch_sign_ * joystick_->x();
        input.yaw_stick = yaw_sign_ * joystick_->y();
        for (int i = 0; i < 3; ++i) { input.position[i] = *positions_[i]; input.velocity[i] = *velocities_[i]; }
        const auto out = logic_.update(input);
        *active_ = out.active; *fault_ = out.fault; *skew_ = out.skew;
        for (int i = 0; i < 3; ++i) {
            *targets_[i] = out.target[i];
            if (out.active) *torques_[i] = pid_[i].update(out.velocity[i] - input.velocity[i]);
            else { pid_[i].reset(); *torques_[i] = 0.0; }
        }
    }
private:
    GantryLogic logic_;
    pid::PidCalculator pid_[3];
    double pitch_sign_, yaw_sign_;
    bool initialized_ = false;
    std::chrono::steady_clock::time_point previous_{};
    InputInterface<std::chrono::steady_clock::time_point> timestamp_;
    InputInterface<Eigen::Vector2d> joystick_;
    InputInterface<rmcs_msgs::Switch> switch_;
    InputInterface<bool> remote_valid_, feedback_valid_;
    InputInterface<double> positions_[3], velocities_[3];
    OutputInterface<double> torques_[3], targets_[3], skew_;
    OutputInterface<bool> active_, fault_;
};
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::gantry::GantryController, rmcs_executor::Component)
