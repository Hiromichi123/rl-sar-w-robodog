#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace jxg {

// Commands and time are supplied as one snapshot by the owner. This class has
// no ROS, FSM, CAN or clock dependencies and is used only by the control thread.
struct DiffDriveConfig {
    double radius = 0.108;
    double track = 0.5685;
    double max_linear = 0.15;
    double max_angular = 0.30;
    double max_wheel = 3.0;
    double wheel_acceleration = 3.0;
    double timeout = 0.5;
    // Logical joint coordinates; the CAN driver already applies motor.dir.
    std::array<double, 4> signs{{1, 1, 1, 1}};

    void validate() const {
        for (double x : {radius, track, max_linear, max_angular, max_wheel,
                         wheel_acceleration, timeout}) {
            if (!std::isfinite(x) || x <= 0) {
                throw std::invalid_argument("invalid differential drive parameter");
            }
        }
        for (double sign : signs) {
            if (sign != 1.0 && sign != -1.0) {
                throw std::invalid_argument("wheel sign must be +1 or -1");
            }
        }
    }
};

struct DiffDriveCommand {
    double vx = 0;
    double vy = 0;
    double wz = 0;
    double received_at = 0;
    bool valid = false;
};

class DiffDriveController {
public:
    explicit DiffDriveController(const DiffDriveConfig& config) : config_(config) {
        config_.validate();
    }

    void enter(double now) {
        reset();
        entered_at_ = now;
        active_ = std::isfinite(now);
    }

    void exit() {
        active_ = false;
        reset();
    }

    // Output order: FL, FR, RL, RR. A zero/invalid/stale command or safety
    // inhibition bypasses the ramp, including any residual previous target.
    std::array<double, 4> step(const DiffDriveCommand& command, double now,
                               double dt, bool permitted) {
        if (!active_ || !permitted || !command.valid ||
            !std::isfinite(now) || !std::isfinite(dt) || dt <= 0 || dt > 0.05 ||
            !std::isfinite(command.vx) || !std::isfinite(command.vy) ||
            !std::isfinite(command.wz) || !std::isfinite(command.received_at) ||
            command.received_at <= entered_at_ || command.received_at > now ||
            now - command.received_at >= config_.timeout ||
            command.vy != 0.0 || (command.vx == 0.0 && command.wz == 0.0)) {
            reset();
            return output_;
        }
        const double vx = std::clamp(command.vx, -config_.max_linear, config_.max_linear);
        const double wz = std::clamp(command.wz, -config_.max_angular, config_.max_angular);
        double left = (vx - wz * config_.track * 0.5) / config_.radius;
        double right = (vx + wz * config_.track * 0.5) / config_.radius;
        const double peak = std::max(std::abs(left), std::abs(right));
        if (peak > config_.max_wheel) {
            const double scale = config_.max_wheel / peak;
            left *= scale;
            right *= scale;
        }
        std::array<double, 4> target{{left, right, left, right}};
        double largest_delta = 0;
        for (unsigned i = 0; i < target.size(); ++i) {
            target[i] *= config_.signs[i];
            largest_delta = std::max(largest_delta, std::abs(target[i] - output_[i]));
        }
        // One scale for the whole vector: starting from rest preserves curvature.
        const double scale = largest_delta > 0
            ? std::min(1.0, config_.wheel_acceleration * dt / largest_delta) : 1.0;
        for (unsigned i = 0; i < target.size(); ++i) {
            output_[i] += (target[i] - output_[i]) * scale;
        }
        return output_;
    }

private:
    void reset() { output_.fill(0); }
    DiffDriveConfig config_;
    std::array<double, 4> output_{};
    double entered_at_ = 0;
    bool active_ = false;
};

}  // namespace jxg
