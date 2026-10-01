#ifndef MOTORMIXER_H
#define MOTORMIXER_H

#include <array>

class MotorMixer {
public:
    static std::array<double, 4> mix(double throttle, double rollCorrection, double pitchCorrection, double yawCorrection);
};

#endif