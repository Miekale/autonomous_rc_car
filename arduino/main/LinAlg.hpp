// Why use a linear algebra library if you can just write on yourself?
// I don't have soft hands

#ifndef LINALG_HPP
#define LINALG_HPP

namespace linalg {

class Vec3 {
public:
    float data[3];

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
    float data[9];

    Mat3();
    Mat3(float flattened[9]);

    Mat3(float a0, float a1, float a2,
     float a3, float a4, float a5,
     float a6, float a7, float a8) {
    data[0]=a0; data[1]=a1; data[2]=a2;
    data[3]=a3; data[4]=a4; data[5]=a5;
    data[6]=a6; data[7]=a7; data[8]=a8;
    }

    float& idx(int i, int j);
    const float& idx(int i, int j) const;

    Mat3 operator+(const Mat3& other) const;
    Mat3 operator-(const Mat3& other) const;
    Mat3 transpose() const;
    Mat3 inverse() const;

    Mat3 operator*(const Mat3& other) const;
    Vec3 operator*(const Vec3& vec3) const;
    Mat3 operator*(float scalar) const;

    static Mat3 Identity();
};

struct FloatPair {
    float first;
    float second;
};

struct StatePair {       // replaces std::pair<Vec3, Mat3>
    Vec3 state;
    Mat3 cov;
};

}

#endif 