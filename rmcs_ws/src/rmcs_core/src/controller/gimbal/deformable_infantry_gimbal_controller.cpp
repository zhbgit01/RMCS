#include "controller/gimbal/two_axis_gimbal_solver.hpp"
#include "controller/pid/pid_calculator.hpp"
#include "yaw_pid_test.hpp"

#include <cmath>
#include <chrono>
#include <stdexcept>
#include <limits>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller::gimbal {

class DeformableInfantryGimbalController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantryGimbalController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        get_parameter_or("yaw_control_enabled", yaw_control_enabled_, true);
        if (yaw_control_enabled_)
            register_output("/gimbal/yaw/control_torque", output_.yaw_control_torque, kNaN);
        get_parameter_or("identification_hold", identification_hold_, false);

        get_parameter_or("yaw_pid_test_enabled", yaw_pid_test_enabled_, false);
        get_parameter_or("yaw_pid_test_torque_limit", yaw_pid_test_torque_limit_, 3.0);
        if (yaw_pid_test_enabled_) {
            register_input("/predefined/timestamp", test_timestamp_);
            register_input("/gimbal/yaw/angle", test_yaw_angle_);
            register_input("/gimbal/yaw/velocity", test_yaw_velocity_);
            if (!yaw_control_enabled_ || !std::isfinite(yaw_pid_test_torque_limit_)
                || yaw_pid_test_torque_limit_ <= 0)
                throw std::runtime_error("Invalid yaw PID test configuration");
        }
        configure_pid("yaw_angle", yaw_angle_pid_);
        configure_pid("yaw_velocity", yaw_velocity_pid_);
        configure_pid("pitch_angle", pitch_angle_pid_);
        configure_pid("pitch_velocity", pitch_velocity_pid_);

        get_parameter("pitch_torque_control", pitch_torque_control_enabled_);
        get_parameter("manual_joystick_sensitivity", joystick_sensitivity_);
        get_parameter("manual_mouse_sensitivity", mouse_sensitivity_);

        get_parameter_or("pitch_gravity_ff_gain", pitch_gravity_ff_gain_, 0.0);
        get_parameter_or("pitch_gravity_ff_phase", pitch_gravity_ff_phase_, 0.0);
        get_parameter_or("yaw_ref_velocity_gain", yaw_ref_velocity_gain_, 1.0);
        get_parameter_or("pitch_ref_velocity_gain", pitch_ref_velocity_gain_, 1.0);
        get_parameter_or("yaw_velocity_ff_gain", yaw_velocity_ff_gain_, 0.0);
        get_parameter_or("yaw_acceleration_ff_gain", yaw_acceleration_ff_gain_, 0.0);
        get_parameter_or("pitch_velocity_ff_gain", pitch_velocity_ff_gain_, 0.0);
        get_parameter_or("pitch_acceleration_ff_gain", pitch_acceleration_ff_gain_, 0.0);
        get_parameter_or("ctrl_hold_pitch_target_angle", ctrl_hold_pitch_target_angle_, 0.0);
    }

    auto update() -> void override {
        const auto switch_right = *input_.switch_right;
        const auto switch_left = *input_.switch_left;
        const auto keyboard = *input_.keyboard;
        double test_dt = update_dt();
        if (yaw_pid_test_enabled_) {
            if (test_last_time_.has_value())
                test_dt = std::chrono::duration<double>(*test_timestamp_ - *test_last_time_).count();
            test_last_time_ = *test_timestamp_;
        }

        using namespace rmcs_msgs;
        if ((switch_left == Switch::UNKNOWN || switch_right == Switch::UNKNOWN)
            || (switch_left == Switch::DOWN && switch_right == Switch::DOWN)) {
            if (yaw_pid_test_enabled_) {
                test_gate_.update(switch_left == Switch::DOWN && switch_right == Switch::DOWN,
                                  false, false, false, test_dt);
                publish_test_status();
            }
            reset_all_controls();
            return;
        }

        if (yaw_pid_test_enabled_) {
            const bool run = switch_left == Switch::MIDDLE && switch_right == Switch::UP;
            const bool was_active = test_gate_.active;
            const bool healthy = std::isfinite(*test_yaw_angle_)
                && std::isfinite(*test_yaw_velocity_) && std::isfinite(*input_.yaw_velocity_imu)
                && std::isfinite(*input_.pitch_angle) && std::isfinite(*input_.pitch_velocity_imu)
                && std::abs(*test_yaw_velocity_) < std::numbers::pi / 2
                && std::abs(*input_.yaw_velocity_imu) < std::numbers::pi / 2
                && (!was_active || std::abs(std::remainder(*test_yaw_angle_ - test_start_angle_,
                       2 * std::numbers::pi)) < 25 * std::numbers::pi / 180);
            test_gate_.update(false, switch_left == Switch::MIDDLE
                && switch_right == Switch::MIDDLE, run, healthy, test_dt);
            if (!test_gate_.active) {
                reset_all_controls();
                publish_test_status();
                return;
            }
            if (!was_active) {
                test_start_angle_ = *test_yaw_angle_;
                test_previous_offset_ = 0;
                gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
                yaw_angle_pid_.reset();
                yaw_velocity_pid_.reset();
            }
            publish_test_status();
        }

        if (!yaw_pid_test_enabled_)
            update_pitch_lock_state(switch_left, switch_right, keyboard);

        if (ctrl_hold_requested()) {
            update_ctrl_hold_control();
        } else {
            deactivate_ctrl_hold();
        }

        const auto auto_aim_active =
            !yaw_pid_test_enabled_ && !identification_hold_ && auto_aim_requested() && input_.auto_aim_should_control.ready()
            && *input_.auto_aim_should_control && input_.auto_aim_control_direction.ready()
            && input_.auto_aim_control_direction->allFinite()
            && !input_.auto_aim_control_direction->isZero();
        const auto angle_error =
            yaw_pid_test_enabled_ ? update_test_control()
                            : auto_aim_active ? update_auto_aim_control() : update_manual_control();

        *output_.yaw_angle_error = angle_error.yaw_angle_error;
        if (!ctrl_hold_active_)
            *output_.pitch_angle_error = angle_error.pitch_angle_error;

        const auto trajectory_ff = trajectory_feedforward(auto_aim_active);

        if (!yaw_control_enabled_) {
            yaw_angle_pid_.reset();
            yaw_velocity_pid_.reset();
        } else if (!std::isfinite(angle_error.yaw_angle_error)) {
            yaw_angle_pid_.reset();
            yaw_velocity_pid_.reset();
            *output_.yaw_control_torque = kNaN;
        } else {
            const auto yaw_velocity_ref =
                yaw_angle_pid_.update(angle_error.yaw_angle_error) + trajectory_ff.yaw_ref_velocity;
            *output_.yaw_control_velocity = yaw_velocity_ref;
            *output_.yaw_control_torque =
                yaw_velocity_pid_.update(yaw_velocity_ref - *input_.yaw_velocity_imu)
                + trajectory_ff.yaw_velocity + trajectory_ff.yaw_acceleration;
        }

        if (yaw_pid_test_enabled_) {
            if (!std::isfinite(*output_.yaw_control_torque)
                || !std::isfinite(angle_error.pitch_angle_error)) {
                test_gate_.fault = true;
                test_gate_.active = false;
                reset_all_controls();
                publish_test_status();
                return;
            }
            *output_.yaw_control_torque = std::clamp(*output_.yaw_control_torque,
                -yaw_pid_test_torque_limit_, yaw_pid_test_torque_limit_);
        }

        if (!ctrl_hold_active_) {
            if (!std::isfinite(angle_error.pitch_angle_error)) {
                pitch_angle_pid_.reset();
                pitch_velocity_pid_.reset();
                *output_.pitch_control_velocity = kNaN;
                *output_.pitch_control_torque = kNaN;
            } else {
                const auto pitch_gravity_ff = pitch_gravity_feedforward();
                const auto pitch_velocity_ref =
                    pitch_angle_pid_.update(angle_error.pitch_angle_error)
                    + trajectory_ff.pitch_ref_velocity;

                if (pitch_torque_control_enabled_) {
                    *output_.pitch_control_velocity = kNaN;
                    *output_.pitch_control_torque =
                        pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity_imu)
                        + pitch_gravity_ff + trajectory_ff.pitch_velocity
                        + trajectory_ff.pitch_acceleration;
                } else {
                    pitch_velocity_pid_.reset();
                    *output_.pitch_control_velocity = pitch_velocity_ref;
                    *output_.pitch_control_torque = kNaN;
                }
            }
        }
    }

private:
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr auto kDefaultDt = 1e-3;

    auto configure_pid(const std::string& prefix, pid::PidCalculator& calculator) -> void {
        get_parameter(prefix + "_integral_min", calculator.integral_min);
        get_parameter(prefix + "_integral_max", calculator.integral_max);
        get_parameter(prefix + "_integral_split_min", calculator.integral_split_min);
        get_parameter(prefix + "_integral_split_max", calculator.integral_split_max);
        get_parameter(prefix + "_output_min", calculator.output_min);
        get_parameter(prefix + "_output_max", calculator.output_max);
    }

    struct Input {
        explicit Input(rmcs_executor::Component& component) {
            component.register_input("/remote/joystick/left", joystick_left);
            component.register_input("/remote/keyboard", keyboard);
            component.register_input("/remote/switch/right", switch_right);
            component.register_input("/remote/switch/left", switch_left);
            component.register_input("/remote/mouse/velocity", mouse_velocity);
            component.register_input("/remote/mouse", mouse);
            component.register_input("/predefined/update_rate", update_rate, false);

            component.register_input("/gimbal/pitch/angle", pitch_angle);
            component.register_input("/gimbal/yaw/velocity_imu", yaw_velocity_imu);
            component.register_input("/gimbal/pitch/velocity_imu", pitch_velocity_imu);

            component.register_input("/auto_aim/should_control", auto_aim_should_control, false);
            component.register_input(
                "/auto_aim/control_direction", auto_aim_control_direction, false);
            component.register_input("/auto_aim/ff_v", auto_aim_ff_v, false);
            component.register_input("/auto_aim/ff_a", auto_aim_ff_a, false);
        }

        InputInterface<Eigen::Vector2d> joystick_left;
        InputInterface<rmcs_msgs::Keyboard> keyboard;
        InputInterface<rmcs_msgs::Switch> switch_right;
        InputInterface<rmcs_msgs::Switch> switch_left;
        InputInterface<Eigen::Vector2d> mouse_velocity;
        InputInterface<rmcs_msgs::Mouse> mouse;
        InputInterface<double> update_rate;

        InputInterface<double> pitch_angle;
        InputInterface<double> yaw_velocity_imu;
        InputInterface<double> pitch_velocity_imu;

        InputInterface<bool> auto_aim_should_control;
        InputInterface<Eigen::Vector3d> auto_aim_control_direction;
        InputInterface<Eigen::Vector3d> auto_aim_ff_v;
        InputInterface<Eigen::Vector3d> auto_aim_ff_a;
    } input_{*this};

    struct Output {
        explicit Output(rmcs_executor::Component& component) {
            component.register_output("/gimbal/yaw/control_velocity", yaw_control_velocity, kNaN);
            component.register_output("/gimbal/yaw/pid_test/time", test_time, 0.0);
            component.register_output("/gimbal/yaw/pid_test/stage", test_stage, -1.0);
            component.register_output("/gimbal/yaw/pid_test/offset", test_offset, 0.0);
            component.register_output("/gimbal/yaw/pid_test/active", test_active, 0.0);
            component.register_output("/gimbal/yaw/pid_test/fault", test_fault, 0.0);
            component.register_output("/gimbal/yaw/pid_test/done", test_done, 0.0);
            component.register_output("/gimbal/yaw/control_angle", yaw_control_angle, kNaN);
            component.register_output(
                "/gimbal/pitch/control_velocity", pitch_control_velocity, kNaN);
            component.register_output("/gimbal/pitch/control_torque", pitch_control_torque, kNaN);
            component.register_output("/gimbal/pitch/control_angle", pitch_control_angle, kNaN);
            component.register_output("/gimbal/yaw/control_angle_error", yaw_angle_error, kNaN);
            component.register_output("/gimbal/pitch/control_angle_error", pitch_angle_error, kNaN);
        }

        OutputInterface<double> yaw_control_velocity;
        OutputInterface<double> test_time, test_stage, test_offset, test_active, test_fault, test_done;
        OutputInterface<double> yaw_control_torque;
        OutputInterface<double> yaw_control_angle;
        OutputInterface<double> pitch_control_velocity;
        OutputInterface<double> pitch_control_torque;
        OutputInterface<double> pitch_control_angle;
        OutputInterface<double> yaw_angle_error;
        OutputInterface<double> pitch_angle_error;
    } output_{*this};

    void publish_test_status() {
        *output_.test_time = test_gate_.elapsed;
        *output_.test_stage = test_gate_.active ? yaw_pid_test_target(test_gate_.elapsed).stage : -1;
        *output_.test_offset = test_gate_.active ? yaw_pid_test_target(test_gate_.elapsed).offset : 0;
        *output_.test_active = test_gate_.active;
        *output_.test_fault = test_gate_.fault;
        *output_.test_done = test_gate_.done;
    }

    auto update_test_control() -> TwoAxisGimbalSolver::AngleError {
        const auto target = yaw_pid_test_target(test_gate_.elapsed);
        const auto error = gimbal_solver_.update(TwoAxisGimbalSolver::SetControlShift{
            target.offset - test_previous_offset_, 0.0});
        test_previous_offset_ = target.offset;
        return error;
    }

    auto ctrl_hold_requested() const -> bool { return pitch_lock_active_; }

    auto update_dt() const -> double {
        if (input_.update_rate.ready() && std::isfinite(*input_.update_rate)
            && *input_.update_rate > 1e-6)
            return 1.0 / *input_.update_rate;
        return kDefaultDt;
    }

    auto manual_yaw_shift() const -> double {
        if (identification_hold_)
            return 0.0;
        return joystick_sensitivity_ * input_.joystick_left->y()
             + mouse_sensitivity_ * input_.mouse_velocity->y();
    }

    auto auto_aim_requested() const -> bool {
        return input_.mouse->right || *input_.switch_right == rmcs_msgs::Switch::UP;
    }

    auto update_auto_aim_control() -> TwoAxisGimbalSolver::AngleError {
        return gimbal_solver_.update(
            TwoAxisGimbalSolver::SetControlDirection{
                OdomImu::DirectionVector{*input_.auto_aim_control_direction}});
    }

    auto update_manual_control() -> TwoAxisGimbalSolver::AngleError {
        if (!gimbal_solver_.enabled())
            return gimbal_solver_.update(TwoAxisGimbalSolver::SetToLevel{});

        const auto yaw_shift = manual_yaw_shift();

        const auto pitch_shift = identification_hold_
                                   ? 0.0
                                   : -joystick_sensitivity_ * input_.joystick_left->x()
                                         + mouse_sensitivity_ * input_.mouse_velocity->x();
        return gimbal_solver_.update(TwoAxisGimbalSolver::SetControlShift{yaw_shift, pitch_shift});
    }

    auto pitch_gravity_feedforward() const -> double {
        if (ctrl_hold_active_)
            return 0.0;
        return pitch_gravity_ff_gain_
             * std::sin(gimbal_solver_.gimbal_world_pitch() - pitch_gravity_ff_phase_);
    }

    struct TrajectoryFeedforward {
        double yaw_ref_velocity = 0.0;
        double pitch_ref_velocity = 0.0;
        double yaw_velocity = 0.0;
        double yaw_acceleration = 0.0;
        double pitch_velocity = 0.0;
        double pitch_acceleration = 0.0;
    };

    auto trajectory_feedforward(bool auto_aim_active) const -> TrajectoryFeedforward {
        if (!auto_aim_active || !input_.auto_aim_ff_v.ready() || !input_.auto_aim_ff_a.ready()
            || !input_.auto_aim_ff_v->allFinite() || !input_.auto_aim_ff_a->allFinite())
            return {};

        const auto ff_v =
            gimbal_solver_.odom_to_yaw_link(OdomImu::DirectionVector{*input_.auto_aim_ff_v});
        const auto ff_a =
            gimbal_solver_.odom_to_yaw_link(OdomImu::DirectionVector{*input_.auto_aim_ff_a});
        return {
            .yaw_ref_velocity = yaw_ref_velocity_gain_ * ff_v->z(),
            .pitch_ref_velocity = pitch_ref_velocity_gain_ * ff_v->y(),
            .yaw_velocity = yaw_velocity_ff_gain_ * ff_v->z(),
            .yaw_acceleration = yaw_acceleration_ff_gain_ * ff_a->z(),
            .pitch_velocity = pitch_velocity_ff_gain_ * ff_v->y(),
            .pitch_acceleration = pitch_acceleration_ff_gain_ * ff_a->y(),
        };
    }

    auto update_pitch_lock_state(
        rmcs_msgs::Switch switch_left, rmcs_msgs::Switch switch_right,
        const rmcs_msgs::Keyboard& keyboard) -> void {
        if (switch_left == rmcs_msgs::Switch::DOWN && switch_right == rmcs_msgs::Switch::UP
            && last_switch_right_ == rmcs_msgs::Switch::MIDDLE) {
            suspension_on_by_switch_ = !suspension_on_by_switch_;
        }

        pitch_lock_active_ = keyboard.ctrl || keyboard.e || suspension_on_by_switch_;
        last_switch_right_ = switch_right;
    }

    auto activate_ctrl_hold() -> void {
        ctrl_hold_active_ = true;
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
    }

    auto deactivate_ctrl_hold() -> void {
        if (!ctrl_hold_active_)
            return;

        ctrl_hold_active_ = false;
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        *output_.pitch_control_angle = kNaN;
    }

    auto update_ctrl_hold_control() -> void {
        if (!ctrl_hold_active_)
            activate_ctrl_hold();

        *output_.yaw_control_angle = kNaN;
        *output_.pitch_control_velocity = kNaN;
        *output_.pitch_control_torque = kNaN;
        *output_.pitch_control_angle = kNaN;

        if (input_.pitch_angle.ready() && std::isfinite(*input_.pitch_angle)) {
            auto pitch_target_error = ctrl_hold_pitch_target_angle_ - *input_.pitch_angle;
            if (pitch_target_error > std::numbers::pi)
                pitch_target_error -= 2 * std::numbers::pi;
            else if (pitch_target_error < -std::numbers::pi)
                pitch_target_error += 2 * std::numbers::pi;

            *output_.pitch_angle_error = pitch_target_error;
            const auto pitch_velocity_ref = pitch_angle_pid_.update(pitch_target_error);
            if (pitch_torque_control_enabled_) {
                *output_.pitch_control_velocity = kNaN;
                *output_.pitch_control_torque =
                    pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity_imu)
                    + pitch_gravity_feedforward();
            } else {
                pitch_velocity_pid_.reset();
                *output_.pitch_control_velocity = pitch_velocity_ref;
                *output_.pitch_control_torque = kNaN;
            }
        }
    }

    auto reset_control_outputs() -> void {
        *output_.yaw_control_velocity = kNaN;
        yaw_angle_pid_.reset();
        yaw_velocity_pid_.reset();
        pitch_angle_pid_.reset();
        pitch_velocity_pid_.reset();
        if (yaw_control_enabled_)
            *output_.yaw_control_torque = kNaN;
        *output_.yaw_control_angle = kNaN;
        *output_.pitch_control_velocity = kNaN;
        *output_.pitch_control_torque = kNaN;
        *output_.pitch_control_angle = kNaN;
    }

    auto reset_all_controls() -> void {
        deactivate_ctrl_hold();
        pitch_lock_active_ = false;
        suspension_on_by_switch_ = false;
        last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
        gimbal_solver_.update(TwoAxisGimbalSolver::SetDisabled{});
        *output_.yaw_angle_error = kNaN;
        *output_.pitch_angle_error = kNaN;
        reset_control_outputs();
    }

    TwoAxisGimbalSolver gimbal_solver_{
        *this, get_parameter("upper_limit").as_double(), get_parameter("lower_limit").as_double()};

    pid::PidCalculator yaw_angle_pid_{
        get_parameter("yaw_angle_kp").as_double(),
        get_parameter("yaw_angle_ki").as_double(),
        get_parameter("yaw_angle_kd").as_double(),
    };
    pid::PidCalculator yaw_velocity_pid_{
        get_parameter("yaw_velocity_kp").as_double(),
        get_parameter("yaw_velocity_ki").as_double(),
        get_parameter("yaw_velocity_kd").as_double(),
    };
    pid::PidCalculator pitch_angle_pid_{
        get_parameter("pitch_angle_kp").as_double(),
        get_parameter("pitch_angle_ki").as_double(),
        get_parameter("pitch_angle_kd").as_double(),
    };
    pid::PidCalculator pitch_velocity_pid_{
        get_parameter("pitch_velocity_kp").as_double(),
        get_parameter("pitch_velocity_ki").as_double(),
        get_parameter("pitch_velocity_kd").as_double(),
    };

    bool yaw_pid_test_enabled_ = false;
    double yaw_pid_test_torque_limit_ = 3.0;
    YawPidTestGate test_gate_;
    InputInterface<std::chrono::steady_clock::time_point> test_timestamp_;
    std::optional<std::chrono::steady_clock::time_point> test_last_time_;
    InputInterface<double> test_yaw_angle_, test_yaw_velocity_;
    double test_start_angle_ = 0, test_previous_offset_ = 0;
    double joystick_sensitivity_ = 0.003;
    double mouse_sensitivity_ = 0.5;
    bool pitch_torque_control_enabled_ = false;
    bool identification_hold_ = false;
    bool yaw_control_enabled_ = true;
    double ctrl_hold_pitch_target_angle_ = 0.0;
    double pitch_gravity_ff_gain_ = 0.0;
    double pitch_gravity_ff_phase_ = 0.0;
    double yaw_ref_velocity_gain_ = 1.0;
    double pitch_ref_velocity_gain_ = 1.0;
    double yaw_velocity_ff_gain_ = 0.0;
    double yaw_acceleration_ff_gain_ = 0.0;
    double pitch_velocity_ff_gain_ = 0.0;
    double pitch_acceleration_ff_gain_ = 0.0;
    bool pitch_lock_active_ = false;
    bool suspension_on_by_switch_ = false;
    rmcs_msgs::Switch last_switch_right_ = rmcs_msgs::Switch::UNKNOWN;
    bool ctrl_hold_active_ = false;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::DeformableInfantryGimbalController, rmcs_executor::Component)
