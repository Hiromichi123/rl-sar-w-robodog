#include "diff_drive_controller.hpp"
#include <cassert>
#include <limits>

int main() {
    jxg::DiffDriveConfig cfg;
    cfg.validate();
    jxg::DiffDriveController drive(cfg);
    drive.enter(10);
    jxg::DiffDriveCommand cmd{.1, 0, 0, 10.01, true};
    auto w = drive.step(cmd, 10.02, .005, true);
    assert(w[0] > 0 && w[0] == w[1] && w[0] == w[2] && w[1] == w[3]);
    assert(w[0] <= cfg.wheel_acceleration * .005 + 1e-10);
    auto zero = [](const auto& x) { for (double v : x) assert(v == 0); };
    zero(drive.step(cmd, 10.52, .005, true));
    cmd.received_at = 10.53; cmd.vx = -.1;
    w = drive.step(cmd, 10.54, .005, true);
    assert(w[0] < 0 && w[0] == w[1]);
    cmd.vx = 0; cmd.wz = .2; cmd.received_at = 11;
    drive.enter(10.9);
    w = drive.step(cmd, 11.01, .005, true);
    assert(w[0] < 0 && w[1] > 0 && w[0] == w[2] && w[1] == w[3]);
    zero(drive.step(cmd, 11.01, .005, false));
    cmd.vy = .01;
    zero(drive.step(cmd, 11.01, .005, true));
    cmd.vy = 0; cmd.vx = std::numeric_limits<double>::quiet_NaN();
    zero(drive.step(cmd, 11.01, .005, true));
    cmd.vx = .1; cmd.wz = std::numeric_limits<double>::infinity();
    zero(drive.step(cmd, 11.01, .005, true));
    cmd.wz = 0;
    drive.enter(12);
    zero(drive.step(cmd, 12.01, .005, true)); // old command
    cmd.received_at = 13;
    zero(drive.step(cmd, 12.01, .005, true)); // future timestamp
    cmd.received_at = 12.01; cmd.vx = 0;
    zero(drive.step(cmd, 12.02, .005, true));
    cmd.vx = 100; cmd.wz = 100;
    for (int i = 0; i < 1000; ++i) {
        cmd.received_at = 14 + i * .005;
        w = drive.step(cmd, cmd.received_at, .005, true);
        for (double v : w) assert(std::abs(v) <= cfg.max_wheel);
    }
    assert(w[0] < w[1]);
    drive.exit();
    zero(drive.step(cmd, cmd.received_at, .005, true));
    for (double bad : {0., -1., std::numeric_limits<double>::infinity()}) {
        cfg.radius = bad;
        bool rejected = false;
        try { cfg.validate(); } catch (const std::invalid_argument&) { rejected = true; }
        assert(rejected);
    }
}
