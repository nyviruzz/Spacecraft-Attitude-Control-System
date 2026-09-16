#include <Vec3.h>
#include <Quaternion.h>
class PDController { 
    private:
        double kp; // proportional gain
        double kd; // derivative gain
        double maxTorque; // saturation limit (N·m)
        Vec3 lastTorque; // for logging

    public:
        PDController(double kp, double kd, double maxTorque);
        Vec3 computeTorque(const Quaternion& q_est, const Vec3& omega_est, const Quaternion& q_target);
        void setGains(double kp, double kd);
        Vec3 getAttitudeError(const Quaternion& q_est, const Quaternion& q_target) const;
        bool isConverged(const Quaternion& q_est, const Quaternion& q_target, double tol_deg=0.1) const;
};
