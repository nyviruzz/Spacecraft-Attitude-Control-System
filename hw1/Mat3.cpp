#include "Mat3.h"
#include <cmath>
#include <stdexcept>

Mat3::Mat3() {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            data[i][j] = 0.0;
        }
    }
}

Mat3::Mat3(const double d[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            data[i][j] = d[i][j];
        }
    }
}

Mat3 Mat3::identity() {
    Mat3 result;
    for (int i = 0; i < 3; i++) {
        result.data[i][i] = 1.0;
    }
    return result;
}

double Mat3::get(int row, int col) const {
    if (row < 0 || row > 2 || col < 0 || col > 2) {
        throw std::out_of_range("Index out of bounds");
    }
    return data[row][col];
}

void Mat3::set(int row, int col, double value) {
    if (row < 0 || row > 2 || col < 0 || col > 2) {
        throw std::out_of_range("Index out of bounds");
    }
    data[row][col] = value;
}

Mat3 Mat3::operator+(const Mat3& other) const {
    Mat3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i][j] = data[i][j] + other.data[i][j];
        }
    }
    return result;
}

Mat3 Mat3::operator-(const Mat3& other) const {
    Mat3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i][j] = data[i][j] - other.data[i][j];
        }
    }
    return result;
}

Mat3 Mat3::operator*(double scalar) const {
    Mat3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i][j] = data[i][j] * scalar;
        }
    }
    return result;
}

Vec3 Mat3::operator*(const Vec3& v) const {
    double x = data[0][0] * v.getX() + data[0][1] * v.getY() + data[0][2] * v.getZ();
    double y = data[1][0] * v.getX() + data[1][1] * v.getY() + data[1][2] * v.getZ();
    double z = data[2][0] * v.getX() + data[2][1] * v.getY() + data[2][2] * v.getZ();
    return Vec3(x, y, z);
}

Mat3 Mat3::operator*(const Mat3& other) const {
    Mat3 result;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i][j] = 0.0;
            for (int k = 0; k < 3; k++) {
                result.data[i][j] += data[i][k] * other.data[k][j];
            }
        }
    }

    return result;
}

Mat3 Mat3::transpose() const {
    Mat3 result;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result.data[i][j] = data[j][i];
        }
    }
    return result;
}

double Mat3::determinant() const {
    double a = data[0][0];
    double b = data[0][1];
    double c = data[0][2];
    double d = data[1][0];
    double e = data[1][1];
    double f = data[1][2];
    double g = data[2][0];
    double h = data[2][1];
    double i = data[2][2];

    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
}

Mat3 Mat3::inverse() const {
    double det = determinant();

    if (std::abs(det) < 1e-12) {
        throw std::runtime_error("Matrix is singular");
    }

    Mat3 cofactor;

    cofactor.data[0][0] =  (data[1][1] * data[2][2] - data[1][2] * data[2][1]);
    cofactor.data[0][1] = -(data[1][0] * data[2][2] - data[1][2] * data[2][0]);
    cofactor.data[0][2] =  (data[1][0] * data[2][1] - data[1][1] * data[2][0]);

    cofactor.data[1][0] = -(data[0][1] * data[2][2] - data[0][2] * data[2][1]);
    cofactor.data[1][1] =  (data[0][0] * data[2][2] - data[0][2] * data[2][0]);
    cofactor.data[1][2] = -(data[0][0] * data[2][1] - data[0][1] * data[2][0]);

    cofactor.data[2][0] =  (data[0][1] * data[1][2] - data[0][2] * data[1][1]);
    cofactor.data[2][1] = -(data[0][0] * data[1][2] - data[0][2] * data[1][0]);
    cofactor.data[2][2] =  (data[0][0] * data[1][1] - data[0][1] * data[1][0]);

    Mat3 adjugate = cofactor.transpose();
    return adjugate * (1.0 / det);
}

double Mat3::trace() const {
    double trace = data[0][0] + data[1][1] + data[2][2];
    return trace; 
}