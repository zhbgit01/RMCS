#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <vector>
#include <cmath>
#include <librmcs/board/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"

namespace rmcs_core::hardware {
class Gantry : public rmcs_executor::Component, public rclcpp::Node,
               public librmcs::board::CBoard::Callback {
    using Clock = std::chrono::steady_clock;
    static long long now_ms() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();
    }
    class Command : public rmcs_executor::Component {
    public:
        explicit Command(Gantry& owner) : owner_(owner) {}
        void update() override { owner_.send(); }
    private:
        Gantry& owner_;
    };
public:
    Gantry() : Node(get_component_name(), rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)),
        command_(create_partner_component<Command>(get_component_name() + "_command", *this)),
        motors_{{*this, *command_, "/gantry/yaw"}, {*this, *command_, "/gantry/left"}, {*this, *command_, "/gantry/right"}} {
        timeout_ms_ = get_parameter("feedback_timeout_ms").as_int();
        torque_limit_ = get_parameter("torque_limit").as_double();
        if (timeout_ms_ <= 0 || !std::isfinite(torque_limit_) || torque_limit_ <= 0 || torque_limit_ > 1.8)
            throw std::invalid_argument("invalid gantry hardware limits");
        for (int i = 0; i < 3; ++i) {
            motors_[i].configure(device::DjiMotor::Config{device::DjiMotor::Type::kM2006, static_cast<uint8_t>(i + 1)}.enable_multi_turn_angle());
            received_[i].store(0);
        }
        register_output("/gantry/feedback_valid", feedback_valid_, false);
        register_output("/remote/valid", remote_valid_, false);
        register_output("/remote/joystick/left", joystick_, Eigen::Vector2d::Zero());
        register_output("/remote/switch/right", enable_switch_, rmcs_msgs::Switch::UNKNOWN);
        RCLCPP_INFO(get_logger(), "[gantry limits] final CAN torque limit=%.3f N.m", torque_limit_);
        board_ = std::make_unique<librmcs::board::CBoard>(*this, get_parameter("board_serial").as_string());
    }
    void update() override {
        // Parse DR16 on the executor thread, avoiding cross-thread Dr16 state access.
        {
            std::lock_guard lock(remote_mutex_);
            if (!remote_packet_.empty()) {
                dr16_.store_status(remote_packet_.data(), remote_packet_.size());
                remote_packet_.clear();
            }
        }
        dr16_.update_status();
        *remote_valid_ = dr16_.valid() && now_ms() - remote_received_.load() <= 500;
        *joystick_ = dr16_.joystick_left();
        *enable_switch_ = dr16_.switch_right();
        *feedback_valid_ = fresh();
        for (int i = 0; i < 3; ++i)
            if (received_[i].load() != 0) motors_[i].update_status();
        const auto now = now_ms();
        if (now - last_diagnostic_ms_ >= 1000) {
            last_diagnostic_ms_ = now;
            std::array<long long, 3> age;
            for (int i = 0; i < 3; ++i) {
                const auto stamp = received_[i].load();
                age[i] = stamp == 0 ? -1 : now - stamp;
            }
            RCLCPP_INFO(get_logger(), "[gantry CAN1] feedback age ms: id1=%lld id2=%lld id3=%lld (-1=never received)", age[0], age[1], age[2]);
        }
    }
private:
    bool fresh() const {
        const auto now = now_ms();
        for (const auto& stamp : received_) {
            auto t = stamp.load();
            if (t == 0 || now - t > timeout_ms_) return false;
        }
        return true;
    }
    void send() {
        device::CanPacket8 packet{};
        const bool allowed = fresh() && *remote_valid_ && *enable_switch_ == rmcs_msgs::Switch::UP;
        for (int i = 0; i < 3; ++i) {
            double torque = motors_[i].control_torque();
            torque = allowed && std::isfinite(torque) ? std::clamp(torque, -torque_limit_, torque_limit_) : 0.0;
            packet.data[i] = motors_[i].generate_command(torque).data;
        }
        board_->start_transmit().can_transmit(Spec::kCans.kCan1, {.can_id = 0x200, .can_data = packet.as_bytes()});
    }
    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (can != Spec::kCans.kCan1 || data.is_extended_can_id || data.is_remote_transmission || data.can_data.size() != 8) return;
        for (int i = 0; i < 3; ++i)
            if (motors_[i].match_then_store_status(data.can_id, data.can_data)) received_[i].store(now_ms());
    }
    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart != Spec::kUarts.kDbus || data.uart_data.size() != 18) return;
        std::lock_guard lock(remote_mutex_);
        remote_packet_.assign(data.uart_data.begin(), data.uart_data.end());
        remote_received_.store(now_ms());
    }
    std::shared_ptr<Command> command_;
    device::DjiMotor motors_[3];
    std::array<std::atomic<long long>, 3> received_{};
    device::Dr16 dr16_;
    std::atomic<long long> remote_received_{0};
    std::mutex remote_mutex_;
    std::vector<std::byte> remote_packet_;
    long long last_diagnostic_ms_ = 0;
    long long timeout_ms_;
    double torque_limit_;
    OutputInterface<bool> feedback_valid_, remote_valid_;
    OutputInterface<Eigen::Vector2d> joystick_;
    OutputInterface<rmcs_msgs::Switch> enable_switch_;
    // Destroy board first to stop callbacks before destroying devices/mutexes.
    std::unique_ptr<librmcs::board::CBoard> board_;
};
}
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::Gantry, rmcs_executor::Component)
