#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "Mat3.h"
#include "Quaternion.h"

class KalmanFilter {
private:
	Quaternion attitudeEstimate;
	Mat3 covarianceP;
	Mat3 processNoiseQ;
	Mat3 measurementNoiseR;
	Mat3 skewSymmetric(const Vec3& v);

public:
	KalmanFilter(const Quaternion& initialAttitude,
				 const Mat3& Q_process,
				 const Mat3& R_measurement);

	void predict(const Vec3& omegaMeasured, double dt);

    void update(const Quaternion& q_measured);
};

#endif
