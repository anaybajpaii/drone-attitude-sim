#include <iostream>
#include <iomanip>
#include "DroneState.h"

int main() {
    const double dt = 0.1;
    const int steps = 100;
    const double radToDeg = 180.0 / 3.14159265358979323846;

    DroneState state;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "time(s)  roll(deg)  pitch(deg)  yaw(deg)\n";

    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        std::cout << std::setw(7) << t << "  "
                  << std::setw(9) << state.getRoll() * radToDeg << "  "
                  << std::setw(10) << state.getPitch() * radToDeg << "  "
                  << std::setw(8) << state.getYaw() * radToDeg << "\n";
        state.update(dt);
    }

    return 0;
}