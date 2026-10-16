#include "../src/controller/gimbal/yaw_pid_test.hpp"
#include <cassert>
#include <initializer_list>
#include <limits>
using namespace rmcs_core::controller::gimbal;
int main() {
    constexpr double deg = std::numbers::pi / 180;
    assert(yaw_pid_test_target(0).offset == 0);
    assert(std::abs(yaw_pid_test_target(3).offset - 5 * deg) < 1e-12);
    assert(yaw_pid_test_target(5).offset == 0);
    assert(std::abs(yaw_pid_test_target(7).offset + 5 * deg) < 1e-12);
    for (double t = 0; t < 57; t += .001) {
        auto target = yaw_pid_test_target(t);
        assert(std::isfinite(target.offset));
        assert(std::abs(target.offset) <= 10 * deg + 1e-12);
        if (t > 27 && t < 43) {
            const auto previous = yaw_pid_test_target(t - .001);
            assert(std::abs(target.offset - previous.offset) <= 10 * deg * .001 + 1e-10);
        }
    }
    assert(yaw_pid_test_target(53).offset == 0);
    YawPidTestGate g;
    g.update(false, false, true, true, .001);
    assert(!g.active); // Cannot start without arming.
    g.update(false, true, false, true, .001);
    g.update(false, false, true, true, .001);
    assert(g.active && g.elapsed == 0);
    for (int i = 0; i < 57000; ++i) g.update(false, false, true, true, .001);
    assert(g.active && g.done && g.elapsed == 56);
    g.update(false, false, true, true, .001);
    assert(g.elapsed == 56); // No automatic repeat.
    g.update(false, false, false, true, .001);
    assert(!g.active);
    g.update(false, true, false, true, .001);
    g.update(false, false, true, true, .001);
    assert(g.active && !g.done && g.elapsed == 0);
    g.update(false, false, true, false, .001);
    assert(g.fault && !g.active);
    g.update(false, true, false, true, .001);
    g.update(false, false, true, true, .001);
    assert(g.fault && !g.active);
    g.update(true, false, false, true, .001);
    assert(!g.fault && !g.active);
    for (double dt : {0.0, -.001, .1, std::numeric_limits<double>::quiet_NaN()}) {
        g.update(true, false, false, true, .001);
        g.update(false, true, false, true, .001);
        g.update(false, false, true, true, dt);
        assert(g.fault && !g.active);
    }
}
