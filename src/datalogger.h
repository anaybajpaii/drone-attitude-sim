#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <fstream>
#include <string>

class DataLogger {
public:
    explicit DataLogger(const std::string& path);
    bool isOpen() const;
    void log(double time, double rollDeg, double pitchDeg, double yawDeg);

private:
    std::ofstream file;
};

#endif