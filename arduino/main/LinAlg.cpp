#include "LinAlg.hpp"

namespace linalg {

Vec3::Vec3() {
    data[0] = 0; data[1] = 0; data[2] = 0;
}

Vec3::Vec3(float one_, float two_, float three_) {
    data[0] = one_; data[1] = two_; data[2] = three_;
}

float& Vec3::operator[](int i) {
    return data[i];
}

const float& Vec3::operator[](int i) const {
    return data[i];
}

Vec3 Vec3::operator+(const Vec3& other) const {
    return Vec3(data[0] + other[0], data[1] + other[1], data[2] + other[2]);
}

Vec3 Vec3::operator-(const Vec3& other) const {
    return Vec3(data[0] - other[0], data[1] - other[1], data[2] - other[2]);
}


// Matrix 3D
Mat3::Mat3() {
    for (int i = 0; i < 9; i++) data[i] = 0.0f;
}

Mat3::Mat3(float flattened[9]) {
    for (int i = 0; i < 9; i++) data[i] = flattened[i];
}

float& Mat3::idx(int i, int j) {
    return data[i*3 + j];
}

const float& Mat3::idx(int i, int j) const {
    return data[i*3 + j];
}

Mat3 Mat3::operator+(const Mat3& other) const {
    Mat3 res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res.idx(i, j) = idx(i,j) + other.idx(i,j);
    return res;
}

Mat3 Mat3::operator-(const Mat3& other) const {
    Mat3 res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res.idx(i, j) = idx(i,j) - other.idx(i,j);
    return res;
}

Mat3 Mat3::operator*(const Mat3& other) const {
    Mat3 res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                res.idx(i, j) += idx(i, k) * other.idx(k, j);
    return res;
}

Vec3 Mat3::operator*(const Vec3& vec3) const {
    Vec3 res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res[i] += idx(i, j) * vec3[j];
    return res;
}

Mat3 Mat3::operator*(float scalar) const {
    Mat3 res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res.idx(i,j) = idx(i,j) * scalar;
    return res;
}

Mat3 Mat3::transpose() const {
    Mat3 res;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            res.idx(j, i) = idx(i, j);
    return res;
}

Mat3 Mat3::Identity() {
    Mat3 res;
    for (int i = 0; i < 3; i++) res.idx(i, i) = 1.0f;
    return res;
}

Mat3 Mat3::inverse() const {
    float c00 =  (idx(1,1)*idx(2,2) - idx(1,2)*idx(2,1));
    float c01 = -(idx(1,0)*idx(2,2) - idx(1,2)*idx(2,0));
    float c02 =  (idx(1,0)*idx(2,1) - idx(1,1)*idx(2,0));

    float c10 = -(idx(0,1)*idx(2,2) - idx(0,2)*idx(2,1));
    float c11 =  (idx(0,0)*idx(2,2) - idx(0,2)*idx(2,0));
    float c12 = -(idx(0,0)*idx(2,1) - idx(0,1)*idx(2,0));

    float c20 =  (idx(0,1)*idx(1,2) - idx(0,2)*idx(1,1));
    float c21 = -(idx(0,0)*idx(1,2) - idx(0,2)*idx(1,0));
    float c22 =  (idx(0,0)*idx(1,1) - idx(0,1)*idx(1,0));

    float det = idx(0,0)*c00 + idx(0,1)*c01 + idx(0,2)*c02;

    if (det == 0.0f) return Mat3{};

    float inv_det = 1.0f / det;

    float flat[9] = {
        c00*inv_det, c10*inv_det, c20*inv_det,
        c01*inv_det, c11*inv_det, c21*inv_det,
        c02*inv_det, c12*inv_det, c22*inv_det
    };
    return Mat3(flat);
}

}  // namespace linalg