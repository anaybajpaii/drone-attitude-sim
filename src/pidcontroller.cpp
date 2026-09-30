#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd), integral(0.0), previousError(0.0) {}

double PIDController::compute(double setpoint, double measured, double dt) {
    double error = setpoint - measured;
    integral += error * dt;
    double derivative = (error - previousError) / dt;
    previousError = error;
    return kp * error + ki * integral + kd * derivative;
}

void PIDController::reset() {
    integral = 0.0;
    previousError = 0.0;
}