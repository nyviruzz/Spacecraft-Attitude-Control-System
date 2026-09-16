#include <Vec3.h>
#include <Quaternion.h>
class Controller {
    private: 
        double kp; // proportional gain
        double kd; // derivative gain
        double maxTorque; // saturation limit (N·m)
        Vec3 lastTorque; // for logging
        
    public:
        virtual ~Controller() = default;
        virtual Vec3 computeTorque(const Quaternion& q_est,
        const Vec3& omega_est,
        const Quaternion& q_target) = 0;
        virtual std::string getName() const = 0;
};
