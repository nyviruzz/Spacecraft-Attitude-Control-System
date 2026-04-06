#ifndef GAUSSIANNOISE_H
#define GAUSSIANNOISE_H

#include <random>
#include "Vec3.h"

class GaussianNoise {
private:
    double mean;
    double stddev;
    std::mt19937 rng;
    std::uniform_real_distribution<double> dist;

public:
    GaussianNoise(double mean, double stddev, unsigned int seed = 42);

    double sample();
    Vec3 sampleVec3();
    void setSeed(unsigned int seed);
};

#endif