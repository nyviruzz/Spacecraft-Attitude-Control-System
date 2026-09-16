#include <cmath>
#include <iomanip>
#include <iostream>

#include "Gyroscope.h"
#include "KalmanFilter.h"
#include "SimLogger.h"
#include "StarTracker.h"

namespace {

Quaternion propagateQuaternion(const Quaternion& q, const Vec3& omega, double dt) {
    Quaternion omegaQ(0.0, omega.getX(), omega.getY(), omega.getZ());
    Quaternion qDot = q * omegaQ;
    Quaternion qNext(
        q.getW() + 0.5 * dt * qDot.getW(),
        q.getX() + 0.5 * dt * qDot.getX(),
        q.getY() + 0.5 * dt * qDot.getY(),
        q.getZ() + 0.5 * dt * qDot.getZ());
    return qNext.normalize();
}

} // namespace

int main() {
    const double dt = 0.01;
    const double durationSec = 10.0;
    const int totalSteps = static_cast<int>(durationSec / dt);

    const Vec3 trueOmega(0.08, -0.04, 0.06);

    Gyroscope gyro(0.01, 0.001, 1);
    StarTracker starTracker(0.002, 10, 2);

    Quaternion trueAttitude;
    Quaternion rawGyroEstimate;
    KalmanFilter kf(trueAttitude, gyro.getCovarianceQ(dt), starTracker.getCovarianceR());

    SimLogger logger("simulation.csv");

    int starUpdates = 0;

    for (int step = 0; step < totalSteps; ++step) {
        double t = step * dt;

        trueAttitude = propagateQuaternion(trueAttitude, trueOmega, dt);

        Vec3 gyroMeasurement = gyro.measure(trueOmega, dt);
        rawGyroEstimate = propagateQuaternion(rawGyroEstimate, gyroMeasurement, dt);

        kf.predict(gyroMeasurement, dt);

        if (starTracker.hasMeasurement(step)) {
            kf.update(starTracker.measure(trueAttitude));
            ++starUpdates;
        }

        logger.log(t, trueAttitude, rawGyroEstimate, kf.getEstimate(), trueOmega, gyroMeasurement);
    }

    Mat3 finalP = kf.getCovariance();
    double covarianceTrace = finalP.get(0, 0) + finalP.get(1, 1) + finalP.get(2, 2);

    logger.printPerformance(durationSec, 1.0 / dt, totalSteps, 10, starUpdates, covarianceTrace);

    return 0;
}