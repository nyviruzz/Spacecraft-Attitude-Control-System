#include "KalmanFilter.h"
#include <cmath>

KalmanFilter::KalmanFilter(const Quaternion& initialAttitude, const Mat3& Q_process, const Mat3& R_measurement)
    : attitudeEstimate(initialAttitude.normalize()),
      covarianceP(Mat3::identity() * 0.01),
      processNoiseQ(Q_process),
      measurementNoiseR(R_measurement),
      lastInnovation(0.0, 0.0, 0.0) {}

void KalmanFilter::predict(const Vec3& omegaMeasured, double dt) {
    if (dt <= 0.0) { return; }

    Quaternion omegaQ(0.0, omegaMeasured.getX(), omegaMeasured.getY(), omegaMeasured.getZ());
    Quaternion qDot = attitudeEstimate * omegaQ;

    Quaternion qNext(
        attitudeEstimate.getW() + 0.5 * dt * qDot.getW(), 
        attitudeEstimate.getX() + 0.5 * dt * qDot.getX(),
        attitudeEstimate.getY() + 0.5 * dt * qDot.getY(),
        attitudeEstimate.getZ() + 0.5 * dt * qDot.getZ()
    );

    attitudeEstimate = qNext.normalize();

    Mat3 F = Mat3::identity() - skewSymmetric(omegaMeasured) * dt;
    covarianceP = F * covarianceP * F.transpose() + processNoiseQ;
}

void KalmanFilter::update(const Quaternion& q_measured) {
    Quaternion delta_q = q_measured * attitudeEstimate.inverse();

    if (delta_q.getW() < 0.0) {
        delta_q = Quaternion(-delta_q.getW(), -delta_q.getX(), -delta_q.getY(), -delta_q.getZ());
    }

    lastInnovation = 2.0 * Vec3(delta_q.getX(), delta_q.getY(), delta_q.getZ());

    Mat3 K = covarianceP * (covarianceP + measurementNoiseR).inverse();
    Vec3 theta_correction = K * lastInnovation;

    Quaternion q_correction(
        1.0,
        theta_correction.getX() / 2.0,
        theta_correction.getY() / 2.0,
        theta_correction.getZ() / 2.0
    );

    attitudeEstimate = (q_correction * attitudeEstimate).normalize();
    covarianceP = (Mat3::identity() - K) * covarianceP;
}

Mat3 KalmanFilter::skewSymmetric(const Vec3& v) {
    double m[3][3] = {
        {  0.0,       -v.getZ(),   v.getY() },
        {  v.getZ(),   0.0,       -v.getX() },
        { -v.getY(),   v.getX(),   0.0      }
    };
    return Mat3(m);
}

Vec3 KalmanFilter::getInnovation() const {
    return lastInnovation;
}

double KalmanFilter::getMahalanobisDistance() const {
    Mat3 S = covarianceP + measurementNoiseR;
    Vec3 Sinv_delta = S.inverse() * lastInnovation;
    double value = lastInnovation.dot(Sinv_delta);

    if (value < 0.0) {
        value = 0.0;
    }

    return sqrt(value);
}

bool KalmanFilter:: isConsistent() const {
    double traceP = covarianceP.trace();
    double d = getMahalanobisDistance();

    const double MAX_TRACE = 1.0;

    return (traceP >= 0.0 && traceP <= MAX_TRACE && d < 3.0);
}

Quaternion KalmanFilter::getEstimate() const {
    return attitudeEstimate;
}

Mat3 KalmanFilter::getCovariance() const {
    return covarianceP;
}

void KalmanFilter::setQ(const Mat3& Q) {
    processNoiseQ = Q;
}

void KalmanFilter::setR(const Mat3& R) {
    measurementNoiseR = R;
}