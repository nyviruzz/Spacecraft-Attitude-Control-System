#ifndef STARTRACKER_H
#define STARTRACKER_H

#include "Quaternion.h"
#include "GaussianNoise.h"

class StarTracker {
private:
    double noiseStddev;
    int updateRate;
    GaussianNoise noiseGen;

public:
    StarTracker(double noiseStddev, int updateRate, unsigned int seed = 2);

    bool hasMeasurement(int step) const;
    Quaternion measure(const Quaternion& trueAttitude);
    Mat3 getCovarianceR() const;
};

#endif