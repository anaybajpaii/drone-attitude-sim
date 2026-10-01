#include "MotorMixer.h"

std::array<double, 4> MotorMixer::mix(double throttle, double rollCorrection, double pitchCorrection, double yawCorrection) {
    std::array<double, 4> motors;
    motors[0] = throttle + rollCorrection + pitchCorrection - yawCorrection;
    motors[1] = throttle - rollCorrection + pitchCorrection + yawCorrection;
    motors[2] = throttle + rollCorrection - pitchCorrection + yawCorrection;
    motors[3] = throttle - rollCorrection - pitchCorrection - yawCorrection;
    return motors;
}