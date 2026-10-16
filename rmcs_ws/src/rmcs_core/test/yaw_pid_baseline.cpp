#include "../src/controller/gimbal/deformable_infantry_gimbal_controller.cpp"
#include <cassert>
#include <iostream>
namespace rmcs_executor {
class Executor {
public:
    template <class T>
    static void bind(Component& c, const std::string& name, T& value) {
        for (auto& i : c.input_list_)
            if (i.name == name) {
                assert(i.type == typeid(T));
                *static_cast<T**>(i.binding) = &value;
                return;
            }
        throw std::runtime_error("missing input " + name);
    }
    template <class T>
    static T value(Component& c, const std::string& name) {
        for (auto& o : c.output_list_)
            if (o.name == name) {
                assert(o.type == typeid(T));
                return *static_cast<T*>(o.binding);
            }
        throw std::runtime_error("missing output " + name);
    }
    static bool has(Component& c, const std::string& name) {
        for (auto& o : c.output_list_)
            if (o.name == name)
                return true;
        return false;
    }
};
} // namespace rmcs_executor
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    using namespace rmcs_description;
    using E = rmcs_executor::Executor;
    using S = rmcs_msgs::Switch;
    rmcs_executor::Component::initializing_component_name = "gimbal_controller";
    rmcs_core::controller::gimbal::DeformableInfantryGimbalController c;
    S left = S::DOWN, right = S::DOWN;
    rmcs_msgs::Keyboard keyboard{};
    rmcs_msgs::Mouse mouse{};
    Eigen::Vector2d joystick = Eigen::Vector2d::Zero(), mouse_velocity = Eigen::Vector2d::Zero();
    double angle = 0, velocity = 0, pitch = 0, pitch_velocity = 0, rate = 1000;
    auto time = std::chrono::steady_clock::now();
    Tf tf;
    tf.get_joint<GimbalCenterLink, YawLink>().set_state(0);
    tf.get_joint<YawLink, PitchLink>().set_state(0);
    E::bind(c, "/tf", tf);
    E::bind(c, "/remote/switch/left", left);
    E::bind(c, "/remote/switch/right", right);
    E::bind(c, "/remote/keyboard", keyboard);
    E::bind(c, "/remote/mouse", mouse);
    E::bind(c, "/remote/joystick/left", joystick);
    E::bind(c, "/remote/mouse/velocity", mouse_velocity);
    E::bind(c, "/predefined/update_rate", rate);
    E::bind(c, "/predefined/timestamp", time);
    E::bind(c, "/gimbal/yaw/angle", angle);
    E::bind(c, "/gimbal/yaw/velocity", velocity);
    E::bind(c, "/gimbal/yaw/velocity_imu", velocity);
    E::bind(c, "/gimbal/pitch/angle", pitch);
    E::bind(c, "/gimbal/pitch/velocity_imu", pitch_velocity);
    auto tick = [&] { time += std::chrono::milliseconds(1); c.update(); };
    auto out = [&] { return E::value<double>(c, "/gimbal/yaw/control_torque"); };
    auto status = [&](const char* key) { return E::value<double>(c, std::string("/gimbal/yaw/pid_test/") + key); };
    tick();
    assert(std::isnan(out()));
    left = S::MIDDLE; right = S::UP;
    tick(); assert(std::isnan(out()));
    left = right = S::MIDDLE; tick();
    right = S::UP; tick();
    assert(status("active") == 1 && out() == 0);
    // Fixed TF: simulated immobile plant. Check scheduled steps and output clamp.
    for (int i = 0; i < 3100; ++i) tick();
    assert(status("stage") == 1);
    assert(std::abs(E::value<double>(c, "/gimbal/yaw/control_angle_error")
                    - 5 * std::numbers::pi / 180) < 1e-6);
    assert(out() > 0 && out() <= 3);
    velocity = -.4; tick(); assert(out() == 3);
    velocity = 0;
    right = S::MIDDLE; tick(); assert(std::isnan(out()));
    right = S::UP; tick(); assert(status("time") == 0);
    velocity = 2; tick(); assert(status("fault") == 1 && std::isnan(out()));
    velocity = 0;
    left = right = S::MIDDLE; tick();
    right = S::UP; tick(); assert(status("fault") == 1 && std::isnan(out()));
    left = right = S::DOWN; tick(); assert(status("fault") == 0);
    left = right = S::MIDDLE; tick(); right = S::UP; tick();
    angle = .5; tick(); assert(status("fault") == 1 && std::isnan(out()));
    angle = 0;
    left = right = S::DOWN; tick();
    left = right = S::MIDDLE; tick(); right = S::UP; tick();
    for (int i = 0; i < 56100; ++i) {
        tick(); assert(std::isfinite(out()) && std::abs(out()) <= 3);
    }
    assert(status("done") == 1 && status("time") == 56);
    left = S::UNKNOWN; tick(); assert(std::isnan(out()) && status("active") == 0);
    left = S::MIDDLE; tick(); assert(std::isnan(out()));
    left = right = S::MIDDLE; tick(); right = S::UP; tick();
    time += std::chrono::milliseconds(100); tick();
    assert(status("fault") == 1 && std::isnan(out()));
    left = right = S::DOWN; tick();
    left = right = S::MIDDLE; tick(); right = S::UP; tick();
    velocity = std::numeric_limits<double>::quiet_NaN(); tick();
    assert(status("fault") == 1 && std::isnan(out()));
    velocity = 0;
    // Only the dedicated run position can drive; direct entry without arming cannot.
    for (S l : {S::UP, S::MIDDLE, S::DOWN, S::UNKNOWN})
        for (S r : {S::UP, S::MIDDLE, S::DOWN, S::UNKNOWN}) {
            left = right = S::DOWN; tick();
            left = l; right = r; tick();
            assert(std::isnan(out()));
        }
    rclcpp::shutdown();
    std::cout << "PASS: PID baseline trigger, schedule, limits, cancel, latch/reset, completion, disconnect\n";
}
