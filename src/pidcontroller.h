#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    double compute(double setpoint, double measured, double dt);
    void reset();

private:
    double kp;
    double ki;
    double kd;
    double integral;
    double previousError;
};

#endif