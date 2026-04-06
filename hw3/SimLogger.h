#ifndef SIMLOGGER_H
#define SIMLOGGER_H

#include <fstream>
#include <string>

#include "Vec3.h"
#include "Quaternion.h"

class SimLogger {
private:
	std::ofstream out;

public:
	explicit SimLogger(const std::string& filename);
	~SimLogger();

	void log(double t,
			 const Quaternion& trueQ,
			 const Quaternion& measQ,
			 const Vec3& trueOmega,
			 const Vec3& measOmega);
};

#endif
