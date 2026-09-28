#include <iostream>
#include <iomanip>
#include "DroneState.h"
#include "DataLogger.h"

int main() {
    const double dt = 0.1;
    const int steps = 100;
    const double radToDeg = 180.0 / 3.14159265358979323846;

    DroneState state;
    DataLogger logger("data/drift.csv");

    if (!logger.isOpen()) {
        std::cerr << "Could not open data/drift.csv for writing\n";
        return 1;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "time(s)  roll(deg)  pitch(deg)  yaw(deg)\n";

    for (int i = 0; i <= steps; ++i) {
        double t = i * dt;
        double rollDeg = state.getRoll() * radToDeg;
        double pitchDeg = state.getPitch() * radToDeg;
        double yawDeg = state.getYaw() * radToDeg;

        std::cout << std::setw(7) << t << "  "
                  << std::setw(9) << rollDeg << "  "
                  << std::setw(10) << pitchDeg << "  "
                  << std::setw(8) << yawDeg << "\n";

        logger.log(t, rollDeg, pitchDeg, yawDeg);
        state.update(dt);
    }

    return 0;
}