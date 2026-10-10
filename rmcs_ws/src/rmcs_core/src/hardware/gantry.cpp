#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <librmcs/board/c_board.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {

// All control runs on Linux. The board forwards CAN/DBUS using compatible firmware.
class Gantry final : public rmcs_executor::Component,
                     public rclcpp::Node,
                     public librmcs::board::CBoard::Callback {
    using Clock = std::chrono::steady_clock;
    class Command final : public rmcs_executor::Component {
    public:
        explicit Command(Gantry& owner) : owner_(owner) {}
        void update() override { owner_.send_command(); }
    private:
        Gantry& owner_;
    };

public:
    Gantry()
        : Node(get_component_name(),
               rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)),
          command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        feedback_timeout_ = positive("feedback_timeout", 0.05);
        remote_timeout_ = positive("remote_timeout", 0.2);
        constexpr std::array names{"left_lift", "right_lift", "yaw"};
        constexpr std::array default_ids{2, 3, 1};
        std::array<int, 3> ids{};
        for (size_t i = 0; i < names.size(); ++i) {
            const std::string name = names[i];
            ids[i] = param<int>(name + "_motor_id", default_ids[i]);
            if (ids[i] < 1 || ids[i] > 8)
                throw std::invalid_argument("gantry: motor IDs must be between 1 and 8");
            for (size_t j = 0; j < i; ++j)
                if (ids[i] == ids[j])
                    throw std::invalid_argument("gantry: duplicate motor ID on CAN1");
            auto config = device::DjiMotor::Config{
                device::DjiMotor::Type::kM2006, static_cast<uint8_t>(ids[i])};
            config.enable_multi_turn_angle();
            if (param<bool>(name + "_reversed", false)) config.set_reversed();
            motors_[i] = std::make_unique<device::DjiMotor>(
                *this, *command_, "/gantry/" + name, config);
            register_output("/gantry/" + name + "/feedback_age", ages_[i], -1.0);
        }
        register_output("/gantry/feedback_valid", feedback_valid_, false);
        register_output("/gantry/remote_valid", remote_valid_, false);
        register_output("/gantry/remote_age", remote_age_, -1.0);
        register_output("/gantry/stick_pitch", stick_pitch_, 0.0);
        register_output("/gantry/stick_yaw", stick_yaw_, 0.0);
        register_output("/remote/joystick/left", joystick_, Eigen::Vector2d::Zero());
        register_output("/remote/switch/right", switch_, rmcs_msgs::Switch::UNKNOWN);
        // Construct last: receive callbacks may start immediately.
        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, param<std::string>("board_serial", ""));
    }

    ~Gantry() override {
        if (board_) {
            try { send_command(true); } catch (...) {}
            board_.reset();
        }
    }

    void update() override {
        const std::scoped_lock lock(mutex_);
        const auto now = Clock::now();
        *feedback_valid_ = true;
        for (size_t i = 0; i < motors_.size(); ++i) {
            *ages_[i] = received_[i] ? elapsed(now, timestamps_[i]) : -1.0;
            *feedback_valid_ = *feedback_valid_ && received_[i]
                            && *ages_[i] <= feedback_timeout_;
            if (received_[i]) motors_[i]->update_status();
        }
        dr16_.update_status();
        *remote_age_ = remote_received_ ? elapsed(now, remote_timestamp_) : -1.0;
        *remote_valid_ = remote_received_ && *remote_age_ <= remote_timeout_ && dr16_.valid();
        *joystick_ = *remote_valid_ ? dr16_.joystick_left() : Eigen::Vector2d::Zero();
        *switch_ = *remote_valid_ ? dr16_.switch_right() : rmcs_msgs::Switch::UNKNOWN;
        *stick_pitch_ = joystick_->x();
        *stick_yaw_ = joystick_->y();
    }

private:
    template <typename T> T param(const std::string& name, T fallback) {
        if (!has_parameter(name)) declare_parameter<T>(name, fallback);
        return get_parameter(name).get_value<T>();
    }
    double positive(const std::string& name, double fallback) {
        const auto value = param<double>(name, fallback);
        if (!std::isfinite(value) || value <= 0)
            throw std::invalid_argument("gantry: invalid " + name);
        return value;
    }
    static double elapsed(Clock::time_point now, Clock::time_point then) {
        return std::chrono::duration<double>(now - then).count();
    }
    void send_command(bool force_stop = false) {
        const std::scoped_lock lock(mutex_);
        const auto now = Clock::now();
        bool safe = !force_stop && remote_received_
                 && elapsed(now, remote_timestamp_) <= remote_timeout_;
        for (size_t i = 0; i < motors_.size(); ++i)
            safe = safe && received_[i] && elapsed(now, timestamps_[i]) <= feedback_timeout_;
        // Every group is assembled once; IDs 1-4 and 5-8 use different command IDs.
        auto builder = board_->start_transmit();
        for (const auto group : {0x200u, 0x1FFu}) {
            device::CanPacket8 packet{uint64_t{0}};
            bool used = false;
            for (const auto& motor : motors_) {
                if (motor->send_id() != group) continue;
                used = true;
                const auto torque = motor->control_torque();
                packet.data[(motor->id() - 1) % 4] =
                    motor->generate_command(safe && std::isfinite(torque) ? torque : 0.0).data;
            }
            if (used) builder.can_transmit(
                Spec::kCans.kCan1, {.can_id = group, .can_data = packet.as_bytes()});
        }
    }
    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can != Spec::kCans.kCan1 || data.is_extended_can_id
            || data.is_remote_transmission || data.can_data.size() != 8) return;
        const std::scoped_lock lock(mutex_);
        for (size_t i = 0; i < motors_.size(); ++i)
            if (motors_[i]->match_then_store_status(data.can_id, data.can_data)) {
                timestamps_[i] = Clock::now();
                received_[i] = true;
            }
    }
    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart != Spec::kUarts.kDbus || data.uart_data.size() != 18) return;
        const std::scoped_lock lock(mutex_);
        dr16_.store_status(data.uart_data.data(), data.uart_data.size());
        remote_timestamp_ = Clock::now();
        remote_received_ = true;
    }

    std::shared_ptr<Command> command_;
    std::array<std::unique_ptr<device::DjiMotor>, 3> motors_;
    device::Dr16 dr16_;
    std::mutex mutex_;
    std::array<Clock::time_point, 3> timestamps_{};
    std::array<bool, 3> received_{};
    Clock::time_point remote_timestamp_{};
    bool remote_received_ = false;
    double feedback_timeout_, remote_timeout_;
    std::array<OutputInterface<double>, 3> ages_;
    OutputInterface<double> remote_age_, stick_pitch_, stick_yaw_;
    OutputInterface<bool> feedback_valid_, remote_valid_;
    OutputInterface<Eigen::Vector2d> joystick_;
    OutputInterface<rmcs_msgs::Switch> switch_;
    std::unique_ptr<librmcs::board::CBoard> board_;
};
} // namespace rmcs_core::hardware

PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gantry, rmcs_executor::Component)
