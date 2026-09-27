#ifndef DRONESTATE_H
#define DRONESTATE_H

class DroneState {
public:
    DroneState();
    void update(double dt);
    double getRoll() const;
    double getPitch() const;
    double getYaw() const;
    double getRollRate() const;
    double getPitchRate() const;
    double getYawRate() const;

private:
    double roll;
    double pitch;
    double yaw;
    double rollRate;
    double pitchRate;
    double yawRate;
    double rollDisturbance;
    double pitchDisturbance;
    double yawDisturbance;
};

#endif