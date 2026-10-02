#include "DroneState.h"

DroneState::DroneState()
    : roll(0.0), pitch(0.0), yaw(0.0),
      rollRate(0.0), pitchRate(0.0), yawRate(0.0),
      rollDisturbance(0.05), pitchDisturbance(-0.03), yawDisturbance(0.02) {}

void DroneState::update(double dt, double rollCorrection, double pitchCorrection, double yawCorrection) {
    rollRate += (rollDisturbance + rollCorrection) * dt;
    pitchRate += (pitchDisturbance + pitchCorrection) * dt;
    yawRate += (yawDisturbance + yawCorrection) * dt;

    roll += rollRate * dt;
    pitch += pitchRate * dt;
    yaw += yawRate * dt;
}

double DroneState::getRoll() const { return roll; }
double DroneState::getPitch() const { return pitch; }
double DroneState::getYaw() const { return yaw; }
double DroneState::getRollRate() const { return rollRate; }
double DroneState::getPitchRate() const { return pitchRate; }
double DroneState::getYawRate() const { return yawRate; }