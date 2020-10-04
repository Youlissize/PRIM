#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <ctime>
#include <string>
using namespace std;
typedef float Real;
typedef long int tIndex;

const int kViewScale = 15;

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

    const T& operator[](const tIndex i) const { assert(i < D); return v[i]; }
    T& operator[](const tIndex i) {
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
    Vector3& increase(const tIndex di, const T2 d) {
        v[di] += static_cast<T>(d); return (*this);
    }
    template<typename T2>
    Vector3 increased(const tIndex di, const T2 d) const {
        return Vector3(*this).increase(di, d);
    }

    Vector3& normalize() { return (x == 0 && y == 0 && z == 0) ? (*this) : (*this) /= length(); }
    Vector3 normalized() const { return Vector3(*this).normalize(); }

    T dotProduct(const Vector3& r) const { return x * r.x + y * r.y + z * r.z; }
    //T crossProduct(const Vector3 &r) const { return x*r.y - y*r.x; }

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



// GLOBAL VARIABLES

vector<Vec3f> triangles;
// vertex data
vector<Vec3f> _X;      // true position at the end of step
vector<Vec3f> _P;      // position approximation during step
vector<Vec3f> _vel;      // velocity
vector<Vec3f> _acc;      // acceleration
Real _defaultMass = 1;      // mass given for each vertex by default
vector<Real> _mass;      // mass of each vertex
vector<Real> _w;        // w[i] = 1/mass[i]   

// simulation
Real _dt = 0.05;                     // time step


// Coefficients
Vec3f  _g = Vec3f(0, -9.8, 0);                    // gravity

string objectFile = "Meshes/example.obj";       // Mesh to import

int solverIteration = 3;      

// END GLOBAL VARIABLES





//Might be useful?
Vec3f VertexInterp( double isolevel,Vec3f p1,Vec3f p2,double valp1,double valp2)
{
   double mu;
   Vec3f p(0,0,0);

   if (fabs(isolevel-valp1) < 0.00001)
      return(p1);
   if (fabs(isolevel-valp2) < 0.00001)
      return(p2);
   if (fabs(valp1-valp2) < 0.00001)
      return(p1);
   mu = (isolevel - valp1) / (valp2 - valp1);
   p.x = p1.x + mu * (p2.x - p1.x);
   p.y = p1.y + mu * (p2.y - p1.y);
   p.z = p1.z + mu * (p2.z - p1.z);

   return(p);
}





class Solver {
public:
    //explicit Solver();
  
  void initScene() {

      for (tIndex i = 0; i < _X.size(); i++) {
          _mass[i] = _defaultMass;
          _w[i] = 1 / _mass[i];
      }

    // make sure for the other quantities
    _vel = vector<Vec3f>(_X.size(), Vec3f(0, 0, 0));
    _acc = vector<Vec3f>(_X.size(), Vec3f(0, 0, 0));

  }
  int c = 0;

  void update() {
    cout << "." << flush;
    _acc = vector<Vec3f>(_X.size(), Vec3f(0, 0, 0));

    // apply External Force
    for (tIndex i = 0; i < vertexCount(); ++i) {
        _acc[i] += _g;
    }
    // update Velocity
#pragma omp parallel for
    for (tIndex i = 0; i < vertexCount(); ++i) {
        _vel[i] += _dt * _w[i] * _acc[i];   // simple forward Euler
    }

    dampVelocities();

    // update Temporary Position
#pragma omp parallel for
    for (tIndex i = 0; i < vertexCount(); ++i) {
        _P[i] = _X[i] + _dt * _vel[i];
    }

    generateCollisionConstraints();
    // Solve constraints
    for (int i = 0; i < solverIteration; i++) {
        projectConstraints();
    }
    
    for (tIndex i = 0; i < vertexCount(); ++i) {
        _vel[i] = (_P[i] - _X[i]) / _dt;
        _X[i] = _P[i];
    }
    velocityUpdate();
    writeInSTL(c);
    c++;

  }

  tIndex vertexCount() const { return _X.size(); }
  const Vec3f& position(const tIndex i) const { return _X[i]; }


private:

  void dampVelocities() {
      // TODO
  }

  void generateCollisionConstraints() {
    // TODO
    }
  
  void projectConstraints() {
      // TODO
  }

  void velocityUpdate() {
      // TODO
  }


  //Compute isovalues to know if we are in or out an object
/* void updateGridIsovalues(){
  const Real sr = _kernel.supportRadius();

  #pragma omp parallel for
  for(tIndex i =0; i < gridcells.size(); i++) {
    for(tIndex k =0; k <8; k++){
      Vec3f xi = gridcells[i].p[k];
      Vec3f posMoy = Vec3f(0,0,0);
      double weightSum = 0;
      const int gi_from = static_cast<int>(xi.x-sr);
      const int gi_to   = static_cast<int>(xi.x+sr)+1;
      const int gj_from = static_cast<int>(xi.y-sr);
      const int gj_to   = static_cast<int>(xi.y+sr)+1;
      const int gk_from = static_cast<int>(xi.z-sr);
      const int gk_to   = static_cast<int>(xi.z+sr)+1;

      for(int gj=max(0, gj_from); gj<min(resY(), gj_to); ++gj) {
        for(int gi=max(0, gi_from); gi<min(resX(), gi_to); ++gi) {
          for(int gk=max(0, gk_from); gk<min(resZ(), gk_to); ++gk){

            const tIndex gidx = idx1d(gi, gj, gk);

            // each particle in nearby cells
            for(size_t ni=0; ni<_pidxInGrid[gidx].size(); ++ni) {
              const tIndex j = _pidxInGrid[gidx][ni];
              const Vec3f &xj = position(j);
              posMoy +=  _kernel.f((xj-xi).length())*xj;
              weightSum+= _kernel.f((xj-xi).length());
            }
          }
        }
      }
      if(weightSum!=0){
      posMoy = posMoy/weightSum;
      gridcells[i].val[k] = (xi-posMoy).length() - _h;
    }
    else gridcells[i].val[k] =1;

    }
    }
  } */

// Allows to write STL file from a list of triangles
  void writeInSTL(int c){
    // TO CHANGE
    /*
    ofstream myfile;
    if( c < 10)
    myfile.open ("water00"  + to_string(c) + ".stl");
    else {if( c >= 10 && c < 100)
    myfile.open ("water0"  + to_string(c) + ".stl");
    else myfile.open ("water"  + to_string(c) + ".stl");}

    myfile << "solid water" << endl;

    for (tIndex j=0; j <n;j++){
      Vec3f p1 = triangles[3*j], p2 = triangles[3*j+1] , p3 = triangles[3*j+2];
      Vec3f B = p2-p1, A = p3-p1;
      Vec3f normal = Vec3f(A.y*B.z-A.z*B.y,A.z*B.x-A.x*B.z,A.x*B.y-A.y*B.x);
      myfile << "facet normal " << normal << endl << "   " << "outer loop" << endl << "      " << "vertex " << p1   << endl << "      " << "vertex " << p2  <<  endl << "      " << "vertex " << p3 << endl << "   " << "endloop" << endl << "endfacet" << endl;
    }
    myfile << "endsolid water" << endl;

    myfile.close();
    */
  }


  };



  void ImportMesh(string file) {
      // TODO
}









int main(int argc, char **argv) {
  int nFrames = 240;
  ImportMesh(objectFile);

  Solver solver;
  solver.initScene();

  for (int i = 0; i < nFrames; i++) {
      solver.update();
  }
  return EXIT_SUCCESS;
}
