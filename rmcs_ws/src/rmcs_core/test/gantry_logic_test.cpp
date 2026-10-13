#include "controller/gantry/gantry_logic.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
using rmcs_core::controller::gantry::GantryLogic;
bool close(double a, double b) { return std::abs(a-b) < 1e-10; }
int main(int argc, char** argv) {
    GantryLogic logic({});
    GantryLogic::Input in;
    in.healthy = true; in.enable = true; in.position = {4, 10, 20};
    assert(!logic.update(in).active); // no automatic startup
    in.enable = false; logic.update(in);
    in.enable = true; auto out = logic.update(in);
    assert(out.active && out.velocity == (std::array<double,3>{0,0,0}));
    in.pitch_stick = 1; out = logic.update(in);
    assert(close(out.target[1]-10, out.target[2]-20));
    assert(out.velocity[1] > 0 && close(out.velocity[1],out.velocity[2]));
    auto pitch = out.target[1];
    in.pitch_stick = 0; in.yaw_stick = 1; out = logic.update(in);
    assert(close(out.target[1],pitch) && out.velocity[0] > 0);
    in.yaw_stick = 0; auto held = logic.update(in);
    assert(out.target == held.target); // centered holds
    in.position[1] += 0.05; out = logic.update(in);
    assert(out.velocity[1] < out.velocity[2]); // higher left is corrected downwards
    in.position[1] += 0.4; out = logic.update(in);
    assert(out.fault && !out.active && out.velocity == (std::array<double,3>{0,0,0}));
    assert(!logic.update(in).active); // needs disable
    in.enable = false; logic.update(in); in.enable = true;
    assert(logic.update(in).active); // relative reference resets only on explicit re-enable
    in.healthy = false; assert(!logic.update(in).active);
    in.healthy = true; assert(!logic.update(in).active);
    in.enable = false; logic.update(in); in.enable = true; logic.update(in);
    in.position[0] = std::numeric_limits<double>::quiet_NaN();
    assert(logic.update(in).fault);
    in.position[0] = 4; in.enable = false; in.pitch_stick = 1; logic.update(in);
    in.enable = true; in.pitch_stick = 0; assert(!logic.update(in).active);
    in.enable = false; logic.update(in); in.enable = true; logic.update(in);
    in.dt = 0.1; assert(!logic.update(in).active);
    GantryLogic backwards({}); GantryLogic::Input down;
    down.healthy = true; backwards.update(down); down.enable = true; backwards.update(down);
    down.pitch_stick = down.yaw_stick = -1;
    auto negative = backwards.update(down);
    for (double v : negative.velocity) assert(v < 0);
    for (double v : negative.velocity) assert(std::abs(v) <= 0.5);
    bool rejected = false;
    try { GantryLogic::Config bad; bad.max_velocity = -1; GantryLogic unused(bad); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    // Closed-loop model with slower/heavier right motor. CSV is optional.
    GantryLogic sim({}); GantryLogic::Input plant; plant.healthy = true;
    sim.update(plant); plant.enable = true; sim.update(plant);
    std::ofstream csv(argc > 1 ? argv[1] : "/tmp/gantry.csv");
    csv << "time,yaw_target,left_target,right_target,yaw,left,right,skew\n";
    double max_skew = 0;
    for (int n=0; n<10000; ++n) {
        plant.pitch_stick = n < 3000 ? 0.6 : 0;
        plant.yaw_stick = n >= 4000 && n < 6000 ? 0.6 : 0;
        auto command = sim.update(plant); assert(command.active);
        max_skew = std::max(max_skew,std::abs(command.skew));
        for (int i=0;i<3;++i) {
            double torque = std::clamp(0.2*(command.velocity[i]-plant.velocity[i]),-0.15,0.15);
            plant.velocity[i] += (torque/(i==2 ? 0.08 : 0.04)-0.3*plant.velocity[i])*plant.dt;
            plant.position[i] += plant.velocity[i]*plant.dt;
        }
        if (n%10==0) csv << n*plant.dt << ',' << command.target[0] << ',' << command.target[1] << ',' << command.target[2] << ',' << plant.position[0] << ',' << plant.position[1] << ',' << plant.position[2] << ',' << command.skew << '\n';
    }
    assert(max_skew < 0.05);
    assert(std::abs(plant.position[1]-plant.position[2]) < 0.002);
    std::cout << "Gantry tests passed; simulated max skew=" << max_skew << " rad\n";
}
