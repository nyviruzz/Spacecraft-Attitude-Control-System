#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "Mat3.h"
#include "GaussianNoise.h"

class Gyroscope {
private:
    Vec3 bias;
    double noiseStddev;
    double biasStddev;
    GaussianNoise noiseGen;
    GaussianNoise biasGen;

public:
    Gyroscope(double noiseStddev, double biasStddev, unsigned int seed = 1);

    Vec3 measure(const Vec3& trueOmega, double dt);
    Mat3 getCovarianceQ(double dt) const;
    Vec3 getBias() const;
};

#endif