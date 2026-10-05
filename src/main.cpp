#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <string>
#include "DroneState.h"
#include "DataLogger.h"
#include "PIDController.h"
#include "MotorMixer.h"

int main(int argc, char* argv[]) {
    const double dt = 0.1;
    const int steps = 100;
    const double radToDeg = 180.0 / 3.14159265358979323846;
    const double degToRad = 3.14159265358979323846 / 180.0;
    const double baseThrottle = 50.0;

    double initRollDeg = 0.0, initPitchDeg = 0.0, initYawDeg = 0.0;
    double rollKp = 2.0, rollKi = 0.1, rollKd = 0.5;
    double pitchKp = 2.0, pitchKi = 0.1, pitchKd = 0.5;
    double yawKp = 2.0, yawKi = 0.1, yawKd = 0.5;
    std::string outputPath = "data/drift.csv";

    if (argc == 13 || argc == 14) {
        initRollDeg = std::atof(argv[1]);
        initPitchDeg = std::atof(argv[2]);
        initYawDeg = std::atof(argv[3]);
        rollKp = std::atof(argv[4]);
        rollKi = std::atof(argv[5]);
        rollKd = std::atof(argv[6]);
        pitchKp = std::atof(argv[7]);
        pitchKi = std::atof(argv[8]);
        pitchKd = std::atof(argv[9]);
        yawKp = std::atof(argv[10]);
        yawKi = std::atof(argv[11]);
        yawKd = std::atof(argv[12]);
        if (argc == 14) {
            outputPath = argv[13];
        }
    } else if (argc != 1) {
        std::cerr << "Usage: " << argv[0]
                  << " [initRollDeg initPitchDeg initYawDeg rollKp rollKi rollKd pitchKp pitchKi pitchKd yawKp yawKi yawKd [outputPath]]\n";
        return 1;
    }

    DroneState state(initRollDeg * degToRad, initPitchDeg * degToRad, initYawDeg * degToRad);
    DataLogger logger(outputPath);

    if (!logger.isOpen()) {
        std::cerr << "Could not open " << outputPath << " for writing\n";
        return 1;
    }

    PIDController rollPID(rollKp, rollKi, rollKd);
    PIDController pitchPID(pitchKp, pitchKi, pitchKd);
    PIDController yawPID(yawKp, yawKi, yawKd);

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

        double rollCorrection = rollPID.compute(0.0, state.getRoll(), dt);
        double pitchCorrection = pitchPID.compute(0.0, state.getPitch(), dt);
        double yawCorrection = yawPID.compute(0.0, state.getYaw(), dt);

        std::array<double, 4> motors = MotorMixer::mix(baseThrottle, rollCorrection, pitchCorrection, yawCorrection);
        (void)motors;

        state.update(dt, rollCorrection, pitchCorrection, yawCorrection);
    }

    return 0;
}