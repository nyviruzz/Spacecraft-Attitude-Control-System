#include <PDController.h>

PDController::PDController(double kp, double kd, double maxTorque) {

}

Vec3 PDController::computeTorque(const Quaternion& q_est, const Vec3& omega_est, const Quaternion& q_target) {
    Quaternion q_error = q_target * q_est;
    if (q_error.getW() < 0) {
        
    }
    Vec3 errorAngle = 2 * Vec3(q_error.getX(), q_error.getY(), q_error.getZ());
    
}