#ifndef SIMLOGGER_H
#define SIMLOGGER_H

#include <fstream>
#include <string>

#include "Vec3.h"
#include "Quaternion.h"

class SimLogger {
private:
	std::ofstream out;
	double rawErrorSqSum;
	double kfErrorSqSum;
	int sampleCount;

	static double attitudeErrorRad(const Quaternion& qTrue, const Quaternion& qEstimate);

public:
	explicit SimLogger(const std::string& filename);
	~SimLogger();

	void log(double t, const Quaternion& trueQ, const Quaternion& rawQ, const Quaternion& estQ,
			 const Vec3& trueOmega, const Vec3& measOmega);

	void printPerformance(double durationSec,
					  double sampleHz,
					  int totalSteps,
					  int trackerEverySteps,
					  int trackerUpdates,
					  double covarianceTrace) const;
};

#endif
