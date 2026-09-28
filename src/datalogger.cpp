#include "DataLogger.h"
#include <filesystem>

DataLogger::DataLogger(const std::string& path) {
    std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::filesystem::create_directories(p.parent_path());
    }
    file.open(path);
    if (file.is_open()) {
        file << "time,roll,pitch,yaw\n";
    }
}

bool DataLogger::isOpen() const {
    return file.is_open();
}

void DataLogger::log(double time, double rollDeg, double pitchDeg, double yawDeg) {
    file << time << "," << rollDeg << "," << pitchDeg << "," << yawDeg << "\n";
}