#ifndef MYVEC3
#define MYVEC3
#include "eigen-3.3.9/Eigen/Dense"

class MyVec3 {
public:
        float x;
        float y;
        float z;

        MyVec3() {x=0.f; y=0.f; z=0.f;}
        MyVec3(float _x, float _y, float _z) {x=_x ; y=_y; z=_z;};
        MyVec3(Vec3f v){ x=v.x; y=v.y; z=v.z; };
        MyVec3(int i) {x=0.f; y=0.f; z=0.f;};

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

        friend istream& operator>>(istream& in, MyVec3& vec) {
            return (in >> vec.x >> vec.y >> vec.z);
        }
        friend ostream& operator<<(ostream& out, const MyVec3& vec) {
            return (out << vec.x << " " << vec.y << " " << vec.z);
        }
};

/*
#define OVERLOAD_OPERATOR(op,ret) ret operator op(const MyVec3 &lhs, const MyVec3 &rhs) { \
return lhs.value op rhs.value; \
}*/

MyVec3 operator+(const float& s, const MyVec3& v)  { return v+s; }
MyVec3 operator-(const float& s, const MyVec3& v)  { return v-s; }
MyVec3 operator*(const float& s, const MyVec3& v)  { return v*s; }
MyVec3 operator/(const float& s, const MyVec3& v)  { return v/s; }

MyVec3 sqrt(MyVec3 v) {
    return MyVec3(std::sqrt(v.x),std::sqrt(v.y),std::sqrt(v.z));
}
MyVec3 abs(MyVec3 v) {
    return MyVec3(abs(v.x),abs(v.y),abs(v.z));
}
MyVec3 abs2(MyVec3 v) {
    return MyVec3(v.x*v.x, v.y*v.y, v.z*v.z);
}

float length(MyVec3 v) {
  return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

bool isfinite(const MyVec3 &) { return true; }

float FLT_MAX = 30000000000.f;
namespace Eigen {
    template<> struct NumTraits<MyVec3>:GenericNumTraits<MyVec3>
        //: NumTraits<float> // permits to get the epsilon, dummy_precision, lowest, highest functions
        {
            typedef MyVec3 Real;
            typedef MyVec3 NonInteger;
            typedef MyVec3 Nested;
            //typedef MyVec3 Literal;

            static inline Real epsilon() { return MyVec3(); }
            static inline Real dummy_precision() { return MyVec3(); }
            static inline int digits10() { return 0; }
            static inline MyVec3 lowest() { return MyVec3(-FLT_MAX,-FLT_MAX,-FLT_MAX);}
            static inline MyVec3 highest() { return MyVec3(FLT_MAX,FLT_MAX,FLT_MAX);}

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
    struct ScalarBinaryOpTraits<MyVec3,float,BinaryOp> { typedef MyVec3 ReturnType;  };

    template<typename BinaryOp>
    struct ScalarBinaryOpTraits<float,MyVec3,BinaryOp> { typedef MyVec3 ReturnType;  };
}



















#endif
