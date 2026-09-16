#ifndef MAT3_H
#define MAT3_H

#include "Vec3.h"

class Mat3 {
private:
	double data[3][3];

public:
	Mat3();
	Mat3(const double d[3][3]);

	static Mat3 identity();

	double get(int row, int col) const;
	void set(int row, int col, double value);

	Mat3 operator+(const Mat3& other) const;
	Mat3 operator-(const Mat3& other) const;
	Mat3 operator*(double scalar) const;
	Vec3 operator*(const Vec3& v) const;
	Mat3 operator*(const Mat3& other) const;

	Mat3 transpose() const;
	double determinant() const;
	Mat3 inverse() const;
    double trace() const;
};

#endif
