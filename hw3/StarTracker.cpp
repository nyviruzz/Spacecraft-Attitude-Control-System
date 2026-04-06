#include "StarTracker.h"

StarTracker::StarTracker(double noiseStddev, int updateRate, unsigned int seed)
    : noiseStddev(noiseStddev),
      updateRate(updateRate),
      noiseGen(0.0, noiseStddev, seed) {}

bool StarTracker::hasMeasurement(int step) const {
    return step % this->updateRate == 0;
}

Quaternion StarTracker::measure(const Quaternion& trueAttitude) {
    double dx = this->noiseGen.sample();
    double dy = this->noiseGen.sample();
    double dz = this->noiseGen.sample();

    Quaternion qNoise(1.0, dx / 2.0, dy / 2.0, dz / 2.0);
    Quaternion qMeas = trueAttitude * qNoise;
    return qMeas.normalize();
}

Mat3 StarTracker::getCovarianceR() const {
    double r = this->noiseStddev * this->noiseStddev;
    return Mat3::identity() * r;
}