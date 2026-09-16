#include "SimLogger.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

double SimLogger::attitudeErrorRad(const Quaternion& qTrue, const Quaternion& qEstimate) {
    Quaternion errorQ = qTrue * qEstimate.inverse();
    double wAbs = std::abs(errorQ.getW());
    if (wAbs > 1.0) {
        wAbs = 1.0;
    }
    return 2.0 * std::acos(wAbs);
}

SimLogger::SimLogger(const std::string& filename)
    : rawErrorSqSum(0.0),
      kfErrorSqSum(0.0),
      sampleCount(0) {
    out.open(filename);

    if (!out.is_open()) {
        throw std::runtime_error("Cannot open log file");
    }

    out << "t,"
        << "true_w,true_x,true_y,true_z,"
        << "raw_w,raw_x,raw_y,raw_z,"
        << "est_w,est_x,est_y,est_z,"
        << "true_ox,true_oy,true_oz,"
        << "meas_ox,meas_oy,meas_oz\n";
}

SimLogger::~SimLogger() {
    if (out.is_open()) {
        out.close();
    }
}

void SimLogger::log(double t, const Quaternion& trueQ, const Quaternion& rawQ, const Quaternion& estQ, const Vec3& trueOmega, const Vec3& measOmega) {
        out << t << ","
        << trueQ.getW() << "," << trueQ.getX() << "," << trueQ.getY() << "," << trueQ.getZ() << ","
        << rawQ.getW() << "," << rawQ.getX() << "," << rawQ.getY() << "," << rawQ.getZ() << ","
        << estQ.getW() << "," << estQ.getX() << "," << estQ.getY() << "," << estQ.getZ() << ","
        << trueOmega.getX() << "," << trueOmega.getY() << "," << trueOmega.getZ() << ","
        << measOmega.getX() << "," << measOmega.getY() << "," << measOmega.getZ() << "\n";

    double rawErr = attitudeErrorRad(trueQ, rawQ);
    double kfErr = attitudeErrorRad(trueQ, estQ);
    rawErrorSqSum += rawErr * rawErr;
    kfErrorSqSum += kfErr * kfErr;
    ++sampleCount;
}

void SimLogger::printPerformance(double durationSec,
                                 double sampleHz,
                                 int totalSteps,
                                 int trackerEverySteps,
                                 int trackerUpdates,
                                 double covarianceTrace) const {
    if (sampleCount == 0) {
        std::cout << "No samples logged.\n";
        return;
    }

    double rawRmsRad = std::sqrt(rawErrorSqSum / sampleCount);
    double kfRmsRad = std::sqrt(kfErrorSqSum / sampleCount);
    double radToDeg = 180.0 / 3.14159265358979323846;
    double rawRmsDeg = rawRmsRad * radToDeg;
    double kfRmsDeg = kfRmsRad * radToDeg;
    double improvement = (kfRmsDeg > 0.0) ? (rawRmsDeg / kfRmsDeg) : 0.0;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Simulation: " << durationSec << " s at " << sampleHz << " Hz (" << totalSteps << " steps)\n";
    std::cout << "Star tracker: every " << trackerEverySteps << " steps (" << trackerUpdates << " updates)\n";
    std::cout << "=== Performance ===\n";
    std::cout << "Raw gyro RMS error: " << rawRmsDeg << " deg\n";
    std::cout << "KF estimate RMS error: " << kfRmsDeg << " deg\n";
    std::cout << "Improvement factor: " << improvement << "x\n";
    std::cout << "Covariance trace (final): " << covarianceTrace << "\n";

    if (kfRmsDeg >= rawRmsDeg) {
        std::cout << "WARNING: filter did not outperform raw gyro in this run.\n";
    }
}