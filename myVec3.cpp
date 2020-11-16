#ifndef MYVEC3
#define MYVEC3
#include "eigen-3.3.8/Eigen/Dense"

class MyVec3 {
public:
        float x;
        float y;
        float z;

        MyVec3() {x=0; y=0; z=0;}
        MyVec3(float _x, float _y, float _z) {x=_x ; y=_x; z=_z;};
        MyVec3(Vec3f v){ x=v.x; y=v.y; z=v.z; };


        // assignment operators
        MyVec3& operator+=(const MyVec3& r) { x += r.x; y += r.y; z += r.z; return *this; }
        MyVec3& operator-=(const MyVec3& r) { x -= r.x; y -= r.y; z -= r.z; return *this; }
        MyVec3& operator*=(const MyVec3& r) { x *= r.x; y *= r.y; z *= r.z; return *this; }
        MyVec3& operator/=(const MyVec3& r) { x /= r.x; y /= r.y; z /= r.z; return *this; }

        MyVec3& operator+=(const float* r) { x += r[0]; y += r[1]; z += r[2]; return *this; }
        MyVec3& operator-=(const float* r) { x -= r[0]; y -= r[1]; z -= r[2]; return *this; }
        MyVec3& operator*=(const float* r) { x *= r[0]; y *= r[1]; z *= r[2]; return *this; }
        MyVec3& operator/=(const float* r) { x /= r[0]; y /= r[1]; z /= r[2]; return *this; }

        MyVec3& operator+=(const float s) { x += s; y += s; z += s; return *this; }
        MyVec3& operator-=(const float s) { x -= s; y -= s; z -= s; return *this; }
        MyVec3& operator*=(const float s) { x *= s; y *= s; z *= s; return *this; }
        MyVec3& operator/=(const float s) { const float d = static_cast<float>(1) / s;
                                            return operator*=(d);}

        // unary operators
        MyVec3 operator+() const { return *this; }
        MyVec3 operator-() const { return MyVec3(-x, -y, -z); }

        // binary operators
        MyVec3 operator+(const MyVec3& r) const { return MyVec3(*this) += r; }
        MyVec3 operator-(const MyVec3& r) const { return MyVec3(*this) -= r; }
        MyVec3 operator*(const MyVec3& r) const { return MyVec3(*this) *= r; }
        MyVec3 operator/(const MyVec3& r) const { return MyVec3(*this) /= r; }

        MyVec3 operator+(const float* r) const { return MyVec3(*this) += r; }
        MyVec3 operator-(const float* r) const { return MyVec3(*this) -= r; }
        MyVec3 operator*(const float* r) const { return MyVec3(*this) *= r; }
        MyVec3 operator/(const float* r) const { return MyVec3(*this) /= r; }

        MyVec3 operator+(const float s) const { return MyVec3(*this) += s; }
        MyVec3 operator-(const float s) const { return MyVec3(*this) -= s; }
        MyVec3 operator*(const float s) const { return MyVec3(*this) *= s; }
        MyVec3 operator/(const float s) const { return MyVec3(*this) /= s; }

        // comparison operators
        bool operator==(const MyVec3& r) const { return ((x == r.x) && (y == r.y) && (z == r.z)); }
        bool operator!=(const MyVec3& r) const { return !((*this) == r); }
};

#define OVERLOAD_OPERATOR(op,ret) ret operator op(const MyVec3 &lhs, const MyVec3 &rhs) { \
return lhs.value op rhs.value; \
    }


MyVec3 sqrt(MyVec3 v) {
    return MyVec3(std::sqrt(v.x),std::sqrt(v.y),std::sqrt(v.z));
}
MyVec3 abs(MyVec3 v) {
    return MyVec3(abs(v.x),abs(v.y),abs(v.z));
}
MyVec3 abs2(MyVec3 v) {
    return MyVec3(v.x*v.x, v.y*v.y, v.z*v.z);
}
bool isfinite(const MyVec3 &) { return true; }

namespace Eigen {
    template<> struct NumTraits<MyVec3>
        : NumTraits<double> // permits to get the epsilon, dummy_precision, lowest, highest functions
        {
            typedef MyVec3 Real;
            typedef MyVec3 NonInteger;
            typedef MyVec3 Nested;

            static inline Real epsilon() { return MyVec3(); }
            static inline Real dummy_precision() { return MyVec3(); }
            static inline int digits10() { return 0; }

            enum {
                IsComplex = 0,
                IsInteger = 0,
                IsSigned = 1,
                RequireInitialization = 1,
                ReadCost = 3,
                AddCost = 9,
                MulCost = 9
            };
        };

    template<typename BinaryOp>
    struct ScalarBinaryOpTraits<MyVec3,double,BinaryOp> { typedef MyVec3 ReturnType;  };

    template<typename BinaryOp>
    struct ScalarBinaryOpTraits<double,MyVec3,BinaryOp> { typedef MyVec3 ReturnType;  };
}


















#endif
