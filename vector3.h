#ifndef VECTOR3
#define VECTOR3
#include<sstream>
#include <math.h>

using namespace std;

typedef float Real;
//typedef long int int;

inline Real square(const Real a) { return a * a; }
inline Real cube(const Real a) { return a * a * a; }
inline Real clamp(const Real v, const Real vmin, const Real vmax) {
    if (v < vmin) return vmin;
    if (v > vmax) return vmax;
    return v;
}

template<typename T>
class Vector3 {
public:
    enum { D = 3 };

    typedef T ValueT;

    union {
        struct { T x; T y; T z; };
        struct { T i; T j; T k; };
        T v[D];
    };

    explicit Vector3(const T& value = 0) : x(value), y(value), z(value) {}
    Vector3(const T& a, const T& b, const T& c) : x(a), y(b), z(c) {}

    // assignment operators
    Vector3& operator+=(const Vector3& r) { x += r.x; y += r.y; z += r.z; return *this; }
    Vector3& operator-=(const Vector3& r) { x -= r.x; y -= r.y; z -= r.z; return *this; }
    Vector3& operator*=(const Vector3& r) { x *= r.x; y *= r.y; z *= r.z; return *this; }
    Vector3& operator/=(const Vector3& r) { x /= r.x; y /= r.y; z /= r.z; return *this; }

    Vector3& operator+=(const T* r) { x += r[0]; y += r[1]; z += r[2]; return *this; }
    Vector3& operator-=(const T* r) { x -= r[0]; y -= r[1]; z -= r[2]; return *this; }
    Vector3& operator*=(const T* r) { x *= r[0]; y *= r[1]; z *= r[2]; return *this; }
    Vector3& operator/=(const T* r) { x /= r[0]; y /= r[1]; z /= r[2]; return *this; }

    Vector3& operator+=(const T s) { x += s; y += s; z += s; return *this; }
    Vector3& operator-=(const T s) { x -= s; y -= s; z -= s; return *this; }
    Vector3& operator*=(const T s) { x *= s; y *= s; z *= s; return *this; }
    Vector3& operator/=(const T s) {
        const T d = static_cast<T>(1) / s; return operator*=(d);
    }

    // unary operators
    Vector3 operator+() const { return *this; }
    Vector3 operator-() const { return Vector3(-x, -y, -z); }

    // binary operators
    Vector3 operator+(const Vector3& r) const { return Vector3(*this) += r; }
    Vector3 operator-(const Vector3& r) const { return Vector3(*this) -= r; }
    Vector3 operator*(const Vector3& r) const { return Vector3(*this) *= r; }
    Vector3 operator/(const Vector3& r) const { return Vector3(*this) /= r; }

    Vector3 operator+(const T* r) const { return Vector3(*this) += r; }
    Vector3 operator-(const T* r) const { return Vector3(*this) -= r; }
    Vector3 operator*(const T* r) const { return Vector3(*this) *= r; }
    Vector3 operator/(const T* r) const { return Vector3(*this) /= r; }

    Vector3 operator+(const T s) const { return Vector3(*this) += s; }
    Vector3 operator-(const T s) const { return Vector3(*this) -= s; }
    Vector3 operator*(const T s) const { return Vector3(*this) *= s; }
    Vector3 operator/(const T s) const { return Vector3(*this) /= s; }

    // comparison operators
    bool operator==(const Vector3& r) const { return ((x == r.x) && (y == r.y) && (z == r.z)); }
    bool operator!=(const Vector3& r) const { return !((*this) == r); }
    // Don't need it ** bool operator<(const Vector3 &r) const { return (x!=r.x) ? x<r.x : y<r.y;

    // cast operator
    template<typename T2>
    operator Vector3<T2>() const {
        return Vector3<T2>(static_cast<T2>(x), static_cast<T2>(y), static_cast<T2>(z));
    }

    const T& operator[](const int i) const { assert(i < D); return v[i]; }
    T& operator[](const int i) {
        return const_cast<T&>(static_cast<const Vector3&>(*this)[i]);
    }

    // special calculative functions

    Vector3& lowerValues(const Vector3& r) {
        x = min(x, r.x); y = min(y, r.y); z = min(z, r.z); return *this;
    }
    Vector3& upperValues(const Vector3& r) {
        x = max(x, r.x); y = max(y, r.y); z = max(z, r.z); return *this;
    }

    template<typename T2>
    Vector3& increase(const int di, const T2 d) {
        v[di] += static_cast<T>(d); return (*this);
    }
    template<typename T2>
    Vector3 increased(const int di, const T2 d) const {
        return Vector3(*this).increase(di, d);
    }

    Vector3& normalize() { return (x == 0 && y == 0 && z == 0) ? (*this) : (*this) /= length(); }
    Vector3 normalized() const { return Vector3(*this).normalize(); }

    T dotProduct(const Vector3& r) const { return x * r.x + y * r.y + z * r.z; }
    //T crossProduct(const Vector3 &r) const { return x*r.y - y*r.x; }
    Vector3 crossProduct(const Vector3 &r) const { return Vector3( y*r.z-z*r.y , z*r.x-x*r.z , x*r.y-y*r.x ); }

    T length() const { return sqrt(lengthSquare()); }
    T lengthSquare() const { return x * x + y * y + z * z; }
    T distanceTo(const Vector3& t) const { return (*this - t).length(); }
    T distanceSquareTo(const Vector3& t) const {
        return (*this - t).lengthSquare();
    }

    T mulAll() const { return x * y * z; }
    T sumAll() const { return x + y + z; }


    T minValue() const { return min(min(x, y), z); }
    T maxValue() const { return max(max(x, y), z); }
    T minAbsValue() const { return min(min(fabs(x), fabs(y)), fabs(z)); }
    T maxAbsValue() const { return max(max(fabs(x), fabs(y)), fabs(z)); }

    Vector3& reflect(const Vector3& n) { return *this = reflected(n); }
    Vector3 reflected(const Vector3& n) const {
        return *this - 2.0 * (this->dotProduct(n)) * n;
    }

    Vector3& mirror(const Vector3& n) { return *this = mirrored(n); }
    Vector3 mirrored(const Vector3& n) const {
        return -(*this) + 2.0 * (this->dotProduct(n)) * n;
    }

    Vector3& project(const Vector3& n) { return *this = projected(n); }
    Vector3 projected(const Vector3& n) const {
        return n * (this->dotProduct(n));
    }

    Vector3& reject(const Vector3& n) { return *this = rejected(n); }
    Vector3 rejected(const Vector3& n) const { return *this - projected(n); }

    friend istream& operator>>(istream& in, Vector3& vec) {
        return (in >> vec.x >> vec.y >> vec.z);
    }
    friend ostream& operator<<(ostream& out, const Vector3& vec) {
        return (out << vec.x << " " << vec.y << " " << vec.z);
    }
};
typedef Vector3<Real> Vec3f;

inline const Vec3f operator*(const Real s, const Vec3f& r) { return r * s; }


#endif
