#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace rmcs_core::controller::gantry {
// Positions are M2006 gearbox-output radians, NOT launcher pitch/yaw angles.
class GantryLogic {
public:
    struct Config {
        double deadzone = 0.08;
        double lift_rate = 0.3, yaw_rate = 0.3;
        double position_kp = 3.0, sync_kp = 2.0;
        double max_velocity = 0.5, max_skew = 0.3;
    };
    struct Input {
        std::array<double, 3> position{}, velocity{}; // yaw, left, right
        double pitch_stick = 0, yaw_stick = 0, dt = 0.001;
        bool healthy = false, enable = false;
    };
    struct Output {
        std::array<double, 3> target{}, velocity{};
        double skew = 0;
        bool active = false, fault = false;
    };
    explicit GantryLogic(Config config) : config_(config) {
        for (double v : {config.deadzone, config.lift_rate, config.yaw_rate,
                         config.position_kp, config.sync_kp, config.max_velocity, config.max_skew})
            if (!std::isfinite(v) || v < 0) throw std::invalid_argument("invalid gantry parameter");
        if (config.deadzone >= 1 || config.max_velocity <= 0 || config.max_skew <= 0)
            throw std::invalid_argument("invalid gantry limits");
    }
    Output update(const Input& in) {
        bool valid = in.healthy && std::isfinite(in.dt) && in.dt > 0 && in.dt <= 0.05
            && std::isfinite(in.pitch_stick) && std::isfinite(in.yaw_stick);
        for (double v : in.position) valid = valid && std::isfinite(v);
        for (double v : in.velocity) valid = valid && std::isfinite(v);
        const bool centered = std::abs(in.pitch_stick) <= config_.deadzone
            && std::abs(in.yaw_stick) <= config_.deadzone;
        // After startup/fault, an explicit disabled-and-centered observation is required.
        if (!valid) { active_ = false; armed_ = false; }
        else if (!in.enable) {
            active_ = false;
            armed_ = centered;
        } else if (!active_ && armed_ && centered) {
            origin_ = in.position;
            lift_target_ = yaw_target_ = 0;
            active_ = true;
        }
        Output out;
        if (active_) {
            const double left = in.position[1] - origin_[1];
            const double right = in.position[2] - origin_[2];
            out.skew = left - right;
            if (std::abs(out.skew) > config_.max_skew) {
                active_ = armed_ = false;
                out.fault = true;
            } else {
                lift_target_ += stick(in.pitch_stick) * config_.lift_rate * in.dt;
                yaw_target_ += stick(in.yaw_stick) * config_.yaw_rate * in.dt;
                // Bound tracking error to avoid accumulating an unreachable target when stalled.
                lift_target_ = std::clamp(lift_target_, (left + right) / 2 - 0.2, (left + right) / 2 + 0.2);
                yaw_target_ = std::clamp(yaw_target_, in.position[0] - origin_[0] - 0.2,
                                       in.position[0] - origin_[0] + 0.2);
                out.velocity[0] = config_.position_kp * (origin_[0] + yaw_target_ - in.position[0]);
                out.velocity[1] = config_.position_kp * (lift_target_ - left) - config_.sync_kp * out.skew;
                out.velocity[2] = config_.position_kp * (lift_target_ - right) + config_.sync_kp * out.skew;
                for (auto& v : out.velocity) v = std::clamp(v, -config_.max_velocity, config_.max_velocity);
            }
        }
        out.target = {origin_[0] + yaw_target_, origin_[1] + lift_target_, origin_[2] + lift_target_};
        out.active = active_;
        out.fault = out.fault || !valid;
        return out;
    }
private:
    double stick(double v) const {
        v = std::clamp(v, -1.0, 1.0);
        return std::abs(v) <= config_.deadzone ? 0 : std::copysign((std::abs(v) - config_.deadzone) / (1 - config_.deadzone), v);
    }
    Config config_;
    std::array<double, 3> origin_{};
    double lift_target_ = 0, yaw_target_ = 0;
    bool active_ = false, armed_ = false;
};
}
