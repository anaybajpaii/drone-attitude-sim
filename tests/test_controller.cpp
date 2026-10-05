#include <iostream>
#include <cmath>
#include "../src/PIDController.h"

bool approxEqual(double a, double b, double epsilon = 0.0001) {
    return std::fabs(a - b) < epsilon;
}

void testProportionalOnly() {
    PIDController pid(2.0, 0.0, 0.0);
    double output = pid.compute(10.0, 0.0, 0.1);
    if (!approxEqual(output, 20.0)) {
        std::cout << "FAIL: testProportionalOnly expected 20.0, got " << output << "\n";
        return;
    }
    std::cout << "PASS: testProportionalOnly\n";
}

void testZeroErrorGivesZeroOutput() {
    PIDController pid(2.0, 0.5, 0.3);
    double output = pid.compute(5.0, 5.0, 0.1);
    if (!approxEqual(output, 0.0)) {
        std::cout << "FAIL: testZeroErrorGivesZeroOutput expected 0.0, got " << output << "\n";
        return;
    }
    std::cout << "PASS: testZeroErrorGivesZeroOutput\n";
}

void testIntegralAccumulates() {
    PIDController pid(0.0, 1.0, 0.0);
    pid.compute(10.0, 0.0, 0.1);
    double output = pid.compute(10.0, 0.0, 0.1);
    if (!approxEqual(output, 2.0)) {
        std::cout << "FAIL: testIntegralAccumulates expected 2.0, got " << output << "\n";
        return;
    }
    std::cout << "PASS: testIntegralAccumulates\n";
}

void testDerivativeRespondsToChange() {
    PIDController pid(0.0, 0.0, 1.0);
    pid.compute(10.0, 0.0, 0.1);
    double output = pid.compute(10.0, 5.0, 0.1);
    if (!approxEqual(output, -50.0)) {
        std::cout << "FAIL: testDerivativeRespondsToChange expected -50.0, got " << output << "\n";
        return;
    }
    std::cout << "PASS: testDerivativeRespondsToChange\n";
}

void testResetClearsState() {
    PIDController pid(0.0, 1.0, 1.0);
    pid.compute(10.0, 0.0, 0.1);
    pid.reset();
    double output = pid.compute(10.0, 0.0, 0.1);
    if (!approxEqual(output, 101.0)) {
        std::cout << "FAIL: testResetClearsState expected 101.0, got " << output << "\n";
        return;
    }
    std::cout << "PASS: testResetClearsState\n";
}

int main() {
    testProportionalOnly();
    testZeroErrorGivesZeroOutput();
    testIntegralAccumulates();
    testDerivativeRespondsToChange();
    testResetClearsState();
    return 0;
}