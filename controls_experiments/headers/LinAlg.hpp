// Why use a linear algebra library if you can just write on yourself?
// I don't have soft hands

#ifndef LINALG_HPP
#define LINALG_HPP

#include <array>

namespace linalg {

class Vec3 {
public:
    std::array<float, 3> data{};

    Vec3();
    Vec3(float one_, float two_, float three_);
    
    float& operator[](int i);
    const float& operator[](int i) const;

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    float dot(const Vec3& other) const;

};

class Mat3 {
public:
    std::array<float, 9> data{};  // i = row*3 + column

    Mat3();
    Mat3(std::array<float, 9> flattened);

    float& idx(int i, int j);
    const float& idx(int i, int j) const;

    Mat3 operator+(const Mat3& other) const;
    Mat3 operator-(const Mat3& other) const;
    Mat3 transpose() const;
    Mat3 inverse() const;

    Mat3 operator*(const Mat3& other) const;
    Vec3 operator*(const Vec3& vec3) const;

    static Mat3 Identity();
};

}

#endif 