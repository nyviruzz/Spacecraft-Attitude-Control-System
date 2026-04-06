#include "SimLogger.h"
#include <stdexcept>

SimLogger::SimLogger(const std::string& filename) {
    out.open(filename);

    if (!out.is_open()) {
        throw std::runtime_error("Cannot open log file");
    }

    out << "t,"
        << "true_w,true_x,true_y,true_z,"
        << "meas_w,meas_x,meas_y,meas_z,"
        << "true_ox,true_oy,true_oz,"
        << "meas_ox,meas_oy,meas_oz\n";
}

SimLogger::~SimLogger() {
    if (out.is_open()) {
        out.close();
    }
}

void SimLogger::log(double t, const Quaternion& trueQ, const Quaternion& measQ,
                    const Vec3& trueOmega, const Vec3& measOmega) {
    out << t << ","
        << trueQ.getW() << "," << trueQ.getX() << "," << trueQ.getY() << "," << trueQ.getZ() << ","
        << measQ.getW() << "," << measQ.getX() << "," << measQ.getY() << "," << measQ.getZ() << ","
        << trueOmega.getX() << "," << trueOmega.getY() << "," << trueOmega.getZ() << ","
        << measOmega.getX() << "," << measOmega.getY() << "," << measOmega.getZ() << "\n";
}