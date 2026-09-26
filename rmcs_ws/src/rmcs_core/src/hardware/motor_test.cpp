#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <numbers>
#include <stdexcept>
#include <string>

#include <librmcs/board/c_board.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {

// Requires RMCS-compatible board firmware, not an arbitrary stock CBoard firmware.
class MotorTest final
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
    using Clock = std::chrono::steady_clock;
    using Pid = controller::pid::PidCalculator;

    class Command final : public rmcs_executor::Component {
    public:
        explicit Command(MotorTest& owner)
            : owner_(owner) {}
        void update() override { owner_.send_command(); }

    private:
        MotorTest& owner_;
    };

public:
    MotorTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this))
        , motor_(*this, *command_, "/motor_test/motor") {
        const auto serial = param<std::string>("board_serial", "");
        const auto mode = param<std::string>("motor_command_mode", "voltage");
        const auto id = param<int>("motor_id", 1);
        if (id < 1 || id > 7 || (mode != "voltage" && mode != "current"))
            throw std::invalid_argument("motor_test: invalid motor ID or command mode");
        auto config = device::DjiMotor::Config{
            mode == "voltage" ? device::DjiMotor::Type::kGM6020Voltage
                              : device::DjiMotor::Type::kGM6020,
            static_cast<uint8_t>(id)};
        if (param<bool>("reversed", false))
            config.set_reversed();
        motor_.configure(config);

        enabled_ = param<bool>("enabled", false);
        max_velocity_ = positive("max_velocity", 1.0);
        angle_range_ = positive("angle_range", 0.5);
        if (angle_range_ >= std::numbers::pi)
            throw std::invalid_argument("motor_test: angle_range must be below pi");
        deadzone_ = param<double>("joystick_deadzone", 0.05);
        if (!std::isfinite(deadzone_) || deadzone_ < 0 || deadzone_ >= 1)
            throw std::invalid_argument("motor_test: invalid joystick deadzone");
        timeout_ = positive("feedback_timeout", 0.1);
        remote_timeout_ = positive("remote_timeout", 0.2);
        const auto limit = positive("output_limit", 0.05);
        velocity_pid_ = make_pid("velocity", limit);
        angle_pid_ = make_pid("angle", max_velocity_);
        cutoff_ = positive("velocity_filter_cutoff", 30.0);
        register_input("/predefined/update_rate", update_rate_);
        register_output("/motor_test/motor/control_torque", output_, 0.0);
        register_output("/motor_test/filtered_velocity", filtered_velocity_, 0.0);
        register_output("/motor_test/target_velocity", target_velocity_, 0.0);
        register_output("/motor_test/target_angle", target_angle_, 0.0);
        register_output("/motor_test/joystick", joystick_, 0.0);
        register_output("/motor_test/mode", mode_output_, 0.0);
        register_output("/motor_test/feedback_valid", feedback_valid_, 0.0);
        register_output("/motor_test/remote_valid", remote_valid_, 0.0);
        // Construct last: callbacks may run as soon as the board connects.
        board_ = std::make_unique<librmcs::board::CBoard>(*this, serial);
    }

    ~MotorTest() override {
        // Best effort only; firmware must enforce its own command timeout too.
        if (board_) {
            try {
                *output_ = 0.0;
                send_command();
            } catch (...) {}
            board_.reset();
        }
    }

    void before_updating() override {
        RCLCPP_INFO(
            get_logger(),
            "Motor test: enabled=%s; right switch DOWN=stop, MIDDLE=speed, UP=angle. Arm in DOWN "
            "with centered right stick.",
            enabled_ ? "true" : "false");
    }

    void update() override {
        // Executor publishes update_rate after before_updating(), before the first tick.
        if (!filter_initialized_) {
            if (!std::isfinite(*update_rate_) || *update_rate_ <= 0)
                throw std::invalid_argument("motor_test: invalid update rate");
            filter_.set_cutoff(cutoff_, *update_rate_);
            filter_initialized_ = true;
        }
        const std::scoped_lock lock(mutex_);
        const auto now = Clock::now();
        const bool fresh = received_motor_ && elapsed(now, motor_received_) <= timeout_;
        const bool remote = received_remote_ && elapsed(now, remote_received_) <= remote_timeout_;
        *feedback_valid_ = fresh ? 1.0 : 0.0;
        dr16_.update_status();
        *remote_valid_ = remote && dr16_.valid() ? 1.0 : 0.0;
        if (received_motor_)
            motor_.update_status();
        *joystick_ = dr16_.joystick_right().x();
        if (fresh)
            *filtered_velocity_ = filter_.update(motor_.velocity());
        if (!enabled_ || !fresh || !*remote_valid_) {
            armed_ = false;
            stop();
            if (!fresh)
                filter_.reset();
            return;
        }

        const auto sw = dr16_.switch_right();
        const double stick = *joystick_;
        if (sw == rmcs_msgs::Switch::DOWN) {
            stop();
            if (std::abs(stick) <= deadzone_)
                armed_ = true;
            return;
        }
        if (!armed_ || (sw != rmcs_msgs::Switch::MIDDLE && sw != rmcs_msgs::Switch::UP)) {
            armed_ = false;
            stop();
            return;
        }
        if (sw != last_mode_) {
            // Require centered stick on entry to either active mode.
            stop();
            if (std::abs(stick) > deadzone_)
                return;
            center_angle_ = motor_.angle();
            last_mode_ = sw;
        }
        const double axis =
            std::abs(stick) <= deadzone_
                ? 0.0
                : std::copysign((std::abs(stick) - deadzone_) / (1.0 - deadzone_), stick);
        *target_angle_ = center_angle_;
        if (sw == rmcs_msgs::Switch::UP) {
            *target_angle_ = center_angle_ + axis * angle_range_;
            const auto error =
                std::remainder(*target_angle_ - motor_.angle(), 2 * std::numbers::pi);
            *target_velocity_ = angle_pid_.update(error);
            *mode_output_ = 2.0;
        } else {
            *target_velocity_ = axis * max_velocity_;
            *mode_output_ = 1.0;
        }
        *output_ = velocity_pid_.update(*target_velocity_ - *filtered_velocity_);
        if (!std::isfinite(*output_)) {
            armed_ = false;
            stop();
        }
    }

private:
    template <typename T>
    T param(const std::string& name, T fallback) {
        if (!has_parameter(name))
            declare_parameter<T>(name, fallback);
        return get_parameter(name).get_value<T>();
    }
    double positive(const std::string& name, double fallback) {
        const auto value = param<double>(name, fallback);
        if (!std::isfinite(value) || value <= 0)
            throw std::invalid_argument("motor_test: " + name + " must be finite and positive");
        return value;
    }
    Pid make_pid(const std::string& prefix, double limit) {
        Pid pid;
        pid.kp = param<double>(prefix + "_kp", 0.0);
        pid.ki = param<double>(prefix + "_ki", 0.0);
        pid.kd = param<double>(prefix + "_kd", 0.0);
        if (!std::isfinite(pid.kp) || !std::isfinite(pid.ki) || !std::isfinite(pid.kd) || pid.kp < 0
            || pid.ki < 0 || pid.kd < 0)
            throw std::invalid_argument("motor_test: invalid PID gains");
        pid.output_min = -limit;
        pid.output_max = limit;
        pid.integral_max = positive(prefix + "_integral_limit", 100.0);
        pid.integral_min = -pid.integral_max;
        return pid;
    }
    static double elapsed(Clock::time_point now, Clock::time_point then) {
        return std::chrono::duration<double>(now - then).count();
    }
    void stop() {
        velocity_pid_.reset();
        angle_pid_.reset();
        last_mode_ = rmcs_msgs::Switch::UNKNOWN;
        *output_ = 0.0;
        *target_velocity_ = 0.0;
        *target_angle_ = received_motor_ ? motor_.angle() : 0.0;
        *mode_output_ = 0.0;
    }
    void send_command() {
        device::CanPacket8 packet{uint64_t{0}};
        packet << motor_;
        board_->start_transmit().can_transmit(
            Spec::kCans.kCan2, {.can_id = motor_.send_id(), .can_data = packet.as_bytes()});
    }
    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can != Spec::kCans.kCan2 || data.is_extended_can_id || data.is_remote_transmission
            || data.can_data.size() != 8)
            return;
        const std::scoped_lock lock(mutex_);
        if (motor_.match_then_store_status(data.can_id, data.can_data)) {
            motor_received_ = Clock::now();
            received_motor_ = true;
        }
    }
    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart != Spec::kUarts.kDbus || data.uart_data.size() != 18)
            return;
        const std::scoped_lock lock(mutex_);
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        remote_received_ = Clock::now();
        received_remote_ = true;
    }

    std::shared_ptr<Command> command_;
    device::DjiMotor motor_;
    device::Dr16 dr16_;
    std::mutex mutex_;
    Clock::time_point motor_received_{}, remote_received_{};
    bool received_motor_ = false, received_remote_ = false;
    bool enabled_ = false, armed_ = false, filter_initialized_ = false;
    double max_velocity_, angle_range_, deadzone_, timeout_, remote_timeout_, cutoff_;
    double center_angle_ = 0.0;
    rmcs_msgs::Switch last_mode_ = rmcs_msgs::Switch::UNKNOWN;
    Pid velocity_pid_, angle_pid_;
    filter::LowPassFilter<> filter_{1.0};
    InputInterface<double> update_rate_;
    OutputInterface<double> output_, filtered_velocity_, target_velocity_, target_angle_;
    OutputInterface<double> joystick_, mode_output_, feedback_valid_, remote_valid_;
    std::unique_ptr<librmcs::board::CBoard> board_;
};

} // namespace rmcs_core::hardware

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MotorTest, rmcs_executor::Component)
