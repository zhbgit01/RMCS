#pragma once
#include <cmath>
#include <numbers>

namespace rmcs_core::controller::gimbal {
// Fixed target offsets in radians; stages: hold, steps, ramps, sine, final hold.
struct YawPidTestTarget {
    double offset;
    int stage;
};
inline auto yaw_pid_test_target(double t) -> YawPidTestTarget {
    constexpr double deg = std::numbers::pi / 180.0;
    if (t < 3) return {0, 0};
    if (t < 27) {
        const int step = static_cast<int>((t - 3) / 2) % 4;
        return {(step == 0 ? 5 : step == 2 ? -5 : 0) * deg, 1};
    }
    if (t < 43) {
        const double p = std::fmod(t - 27, 8.0);
        // 0 -> +10 -> 0 -> -10 -> 0, with one-second stops at extrema/center.
        double angle = 0;
        if (p < 1) angle = 10 * p;
        else if (p < 2) angle = 10;
        else if (p < 3) angle = 10 * (3 - p);
        else if (p < 4) angle = 0;
        else if (p < 5) angle = -10 * (p - 4);
        else if (p < 6) angle = -10;
        else if (p < 7) angle = -10 * (7 - p);
        return {angle * deg, 2};
    }
    if (t < 53) return {5 * deg * std::sin(std::numbers::pi * (t - 43)), 3};
    return {0, 4};
}

// Switch encoding is deliberately independent of ROS for exhaustive state tests.
class YawPidTestGate {
public:
    bool active = false, armed = false, fault = false, done = false;
    double elapsed = 0;
    void update(bool both_down, bool both_middle, bool run, bool healthy, double dt) {
        if (both_down) {
            active = armed = fault = done = false;
            elapsed = 0;
            return;
        }
        if (fault) return;
        if (!run) {
            active = false;
            armed = both_middle;
            return;
        }
        if (!healthy || !std::isfinite(dt) || dt <= 0 || dt > .05) {
            active = armed = false;
            fault = true;
            return;
        }
        if (!active && armed) {
            elapsed = 0;
            active = true;
            armed = done = false;
        } else if (active && !done) {
            elapsed += dt;
            if (elapsed >= 56) {
                elapsed = 56;
                done = true;
            }
        }
    }
};
} // namespace rmcs_core::controller::gimbal
