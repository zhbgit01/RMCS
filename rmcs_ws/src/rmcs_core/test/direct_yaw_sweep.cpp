#include "../src/controller/chassis/deformable_chassis.cpp"
#include "../src/controller/gimbal/deformable_infantry_gimbal_controller.cpp"
#include "../src/identification/swept_frequency_controller.cpp"
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
    using C = rmcs_core::controller::identification::SweptFrequencyController;
    using E = rmcs_executor::Executor;
    using S = rmcs_msgs::Switch;
    {
        rmcs_executor::Component::initializing_component_name = "yaw_swept_frequency_controller";
        C c;
        rmcs_executor::Component::initializing_component_name = "gimbal_controller";
        rmcs_core::controller::gimbal::DeformableInfantryGimbalController gimbal;
        assert(!E::has(gimbal, "/gimbal/yaw/control_torque"));
        assert(!E::has(gimbal, "/gimbal/yaw/base_control_torque"));
        assert(E::has(gimbal, "/gimbal/pitch/control_torque"));
        rmcs_executor::Component::initializing_component_name = "chassis_controller";
        rmcs_core::controller::chassis::DeformableChassis chassis;
        S left = S::DOWN, right = S::DOWN;
        size_t count = 1;
        E::bind(chassis, "/remote/switch/left", left);
        E::bind(chassis, "/remote/switch/right", right);
        for (S l : {S::UP, S::MIDDLE, S::DOWN, S::UNKNOWN})
            for (S r : {S::UP, S::MIDDLE, S::DOWN, S::UNKNOWN}) {
                left = l;
                right = r;
                auto reset = E::value<size_t>(chassis, "/chassis/deformable/reset_count");
                chassis.update();
                assert(E::value<size_t>(chassis, "/chassis/deformable/reset_count") == reset + 1);
                assert(
                    std::isnan(
                        E::value<rmcs_description::BaseLink::DirectionVector>(
                            chassis, "/chassis/control_velocity")
                            .vector[0]));
            }
        left = right = S::DOWN;
        auto time = std::chrono::steady_clock::now();
        const auto start = time;
        double torque = 0, velocity = 0, angle = 0;
        E::bind(c, "/remote/switch/left", left);
        E::bind(c, "/remote/switch/right", right);
        E::bind(c, "/predefined/update_count", count);
        E::bind(c, "/predefined/timestamp", time);
        E::bind(c, "/gimbal/yaw/torque", torque);
        E::bind(c, "/gimbal/yaw/velocity", velocity);
        E::bind(c, "/gimbal/yaw/angle", angle);
        auto output = [&] { return E::value<double>(c, "/gimbal/yaw/control_torque"); };
        auto active = [&] { return E::value<double>(c, "/gimbal/yaw/sweep/active"); };
        auto fault = [&] { return E::value<double>(c, "/gimbal/yaw/sweep/fault"); };
        auto tick = [&](double seconds) {
            time = start
                 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                       std::chrono::duration<double>(seconds));
            ++count;
            c.update();
        };
        c.before_updating();
        tick(0);
        assert(std::isnan(output()));
        left = right = S::MIDDLE;
        tick(.005);
        assert(std::isnan(output()));
        right = S::UP;
        tick(.010);
        assert(active() == 1 && output() == 0);
        tick(.510);
        assert(
            std::isfinite(output())
            && std::abs(output()) <= c.get_parameter("torque_limit").as_double() + 1e-6);
        right = S::MIDDLE;
        tick(.520);
        assert(active() == 0 && std::isnan(output()));
        right = S::UP;
        tick(.525);
        assert(active() == 1);
        angle = .4;
        tick(.528);
        assert(active() == 1 && fault() == 0 && std::isfinite(output()));
        velocity = 2;
        tick(.530);
        assert(fault() == 1 && std::isnan(output()) && active() == 0);
        angle = 0;
        velocity = 0;
        left = right = S::MIDDLE;
        tick(.535);
        right = S::UP;
        tick(.536);
        assert(std::isnan(output()) && fault() == 1);
        left = right = S::DOWN;
        tick(.540);
        assert(fault() == 0 && std::isnan(output()));
        left = right = S::MIDDLE;
        tick(.545);
        right = S::UP;
        tick(.550);
        assert(active() == 1);
        velocity = 2;
        tick(.555);
        assert(fault() == 1 && std::isnan(output()));
        velocity = 0;
        left = right = S::DOWN;
        tick(.560);
        left = right = S::MIDDLE;
        tick(.565);
        right = S::UP;
        tick(.570);
        tick(60.570);
        assert(active() == 0 && std::isnan(output()));
        tick(60.575);
        assert(active() == 0 && std::isnan(output()));
        left = right = S::MIDDLE;
        tick(60.580);
        right = S::UP;
        tick(60.585);
        assert(active() == 1);
        left = S::UNKNOWN;
        tick(60.590);
        assert(std::isnan(output()) && active() == 0);
        left = right = S::MIDDLE;
        tick(60.595);
        right = S::UP;
        torque = std::numeric_limits<double>::quiet_NaN();
        tick(60.600);
        assert(fault() == 1 && std::isnan(output()));
    }
    rclcpp::shutdown();
    std::cout
        << "PASS: sole yaw owner, pitch retained, chassis disabled in all 16 switch states, direct "
           "excitation, idle/exit/end disabled, protection latch/reset and disconnect\n";
}
