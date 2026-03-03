#include "LinAlg.hpp"
#include <array>

using namespace linalg;

Vec3::Vec3() {
    data = std::array<float, 3>{0,0,0};
}

Vec3::Vec3(float one_, float two_, float three_) {
    data = std::array<float, 3>{one_, two_, three_};
}

float& Vec3::operator[](int i) {
    return data[i];
}

const float& Vec3::operator[](int i) const {
    return data[i];
}

Vec3 Vec3::operator+(const Vec3& other) const {
    Vec3 res = Vec3(data[0] + other[0], data[1] + other[1], data[2] + other[2]);
    return res;
}

Vec3 Vec3::operator-(const Vec3& other) const {
    Vec3 res = Vec3(data[0] - other[0], data[1] - other[1], data[2] - other[2]);
    return res;
}


// Matrix 3D
Mat3::Mat3(std::array<float, 9> flattened) {
    data = flattened;
}

float& Mat3::idx(int i, int j) {
    return data[i*3 + j];
}
const float& Mat3::idx(int i, int j) const {
    return data[i*3 + j];
}

Mat3 Mat3::operator+(const Mat3& other) const {
    Mat3 res;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res.idx(i, j) = idx(i,j) + other.idx(i,j);
        }
    }

    return res;
}

Mat3 Mat3::operator-(const Mat3& other) const {
    Mat3 res;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res.idx(i, j) = idx(i,j) - other.idx(i,j);
        }
    }

    return res;
}

Mat3 Mat3::operator*(const Mat3& other) const {
    // Simple n^3 nested for-loop implementation
    Mat3 res;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                res.idx(i, j) += idx(i, k) * other.idx(k, j);
            }
        }
    }

    return res;
}

Vec3 Mat3::operator*(const Vec3& vec3) const {
    Vec3 res;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res[i] += idx(i, j) + vec3[j];
        }
    }
    return res;
}


Mat3 Mat3::transpose() const {
    Mat3 res;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            res.idx(j, i) = this->idx(i, j);
        }
    }

    return res;
}

Mat3 Mat3::Identity() {
    Mat3 res;
    for (int i = 0; i < 3; i++) {
        res.idx(i, i) = 1;
    }

    return res;
}
