#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <ctime>
using namespace std;
typedef float Real;
typedef long int tIndex;

const int kViewScale = 15;

inline Real square(const Real a) { return a*a; }
inline Real cube(const Real a) { return a*a*a; }
inline Real clamp(const Real v, const Real vmin, const Real vmax) {
  if(v<vmin) return vmin;
  if(v>vmax) return vmax;
  return v;
}

template<typename T>
class Vector3 {
public:
  enum { D = 3 };

  typedef T ValueT;

  union {
    struct { T x; T y; T z;};
    struct { T i; T j; T k;};
    T v[D];
  };

  explicit Vector3(const T &value=0) : x(value), y(value), z(value) {}
  Vector3(const T &a, const T &b, const T &c) : x(a), y(b), z(c) {}

  // assignment operators
  Vector3& operator+=(const Vector3 &r) { x+=r.x; y+=r.y; z+= r.z; return *this; }
  Vector3& operator-=(const Vector3 &r) { x-=r.x; y-=r.y; z-= r.z; return *this; }
  Vector3& operator*=(const Vector3 &r) { x*=r.x; y*=r.y; z*= r.z; return *this; }
  Vector3& operator/=(const Vector3 &r) { x/=r.x; y/=r.y; z/= r.z; return *this; }

  Vector3& operator+=(const T *r) { x+=r[0]; y+=r[1]; z+=r[2]; return *this; }
  Vector3& operator-=(const T *r) { x-=r[0]; y-=r[1]; z-=r[2]; return *this; }
  Vector3& operator*=(const T *r) { x*=r[0]; y*=r[1]; z*=r[2]; return *this; }
  Vector3& operator/=(const T *r) { x/=r[0]; y/=r[1]; z/=r[2]; return *this; }

  Vector3& operator+=(const T s) { x+=s; y+=s; z+=s; return *this; }
  Vector3& operator-=(const T s) { x-=s; y-=s; z-=s; return *this; }
  Vector3& operator*=(const T s) { x*=s; y*=s; z*=s; return *this; }
  Vector3& operator/=(const T s) {
    const T d=static_cast<T>(1)/s; return operator*=(d);
  }

  // unary operators
  Vector3 operator+() const { return *this; }
  Vector3 operator-() const { return Vector3(-x, -y, -z); }

  // binary operators
  Vector3 operator+(const Vector3 &r) const { return Vector3(*this)+=r; }
  Vector3 operator-(const Vector3 &r) const { return Vector3(*this)-=r; }
  Vector3 operator*(const Vector3 &r) const { return Vector3(*this)*=r; }
  Vector3 operator/(const Vector3 &r) const { return Vector3(*this)/=r; }

  Vector3 operator+(const T *r) const { return Vector3(*this)+=r; }
  Vector3 operator-(const T *r) const { return Vector3(*this)-=r; }
  Vector3 operator*(const T *r) const { return Vector3(*this)*=r; }
  Vector3 operator/(const T *r) const { return Vector3(*this)/=r; }

  Vector3 operator+(const T s) const { return Vector3(*this)+=s; }
  Vector3 operator-(const T s) const { return Vector3(*this)-=s; }
  Vector3 operator*(const T s) const { return Vector3(*this)*=s; }
  Vector3 operator/(const T s) const { return Vector3(*this)/=s; }

  // comparison operators
  bool operator==(const Vector3 &r) const { return ((x==r.x) && (y==r.y) && (z ==r.z)); }
  bool operator!=(const Vector3 &r) const { return !((*this)==r); }
  // Don't need it ** bool operator<(const Vector3 &r) const { return (x!=r.x) ? x<r.x : y<r.y;

  // cast operator
  template<typename T2>
  operator Vector3<T2>() const {
    return Vector3<T2>(static_cast<T2>(x), static_cast<T2>(y), static_cast<T2>(z));
  }

  const T& operator[](const tIndex i) const { assert(i<D); return v[i]; }
  T& operator[](const tIndex i) {
    return const_cast<T &>(static_cast<const Vector3 &>(*this)[i]);
  }

  // special calculative functions

  Vector3& lowerValues(const Vector3 &r) {
    x = min(x, r.x); y = min(y, r.y); z = min(z, r.z); return *this;
  }
  Vector3& upperValues(const Vector3 &r) {
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

  Vector3& normalize() { return (x==0 && y==0 && z==0) ? (*this) : (*this)/=length(); }
  Vector3 normalized() const { return Vector3(*this).normalize(); }

  T dotProduct(const Vector3 &r) const { return x*r.x + y*r.y + z*r.z;  }
  //T crossProduct(const Vector3 &r) const { return x*r.y - y*r.x; }

  T length() const { return sqrt(lengthSquare()); }
  T lengthSquare() const { return x*x + y*y + z*z; }
  T distanceTo(const Vector3 &t) const { return (*this-t).length(); }
  T distanceSquareTo(const Vector3 &t) const {
    return (*this-t).lengthSquare();
  }

  T mulAll() const { return x*y*z; }
  T sumAll() const { return x+y+z; }


  T minValue() const { return min(min(x, y),z); }
  T maxValue() const { return max(max(x, y),z); }
  T minAbsValue() const { return min(min(fabs(x), fabs(y)),fabs(z)); }
  T maxAbsValue() const { return max(max(fabs(x), fabs(y)),fabs(z)); }

  Vector3& reflect(const Vector3 &n) { return *this = reflected(n); }
  Vector3 reflected(const Vector3 &n) const {
    return *this - 2.0*(this->dotProduct(n))*n;
  }

  Vector3& mirror(const Vector3 &n) { return *this = mirrored(n); }
  Vector3 mirrored(const Vector3 &n) const {
    return -(*this) + 2.0*(this->dotProduct(n))*n;
  }

  Vector3& project(const Vector3 &n) { return *this = projected(n); }
  Vector3 projected(const Vector3 &n) const {
    return n * (this->dotProduct(n));
  }

  Vector3& reject(const Vector3 &n) { return *this = rejected(n); }
  Vector3 rejected(const Vector3 &n) const { return *this - projected(n); }

  friend istream& operator>>(istream &in, Vector3 &vec) {
    return (in >> vec.x >> vec.y >> vec.z);
  }
  friend ostream& operator<<(ostream &out, const Vector3 &vec) {
    return (out << vec.x << " " << vec.y << " " << vec.z);
  }
};
typedef Vector3<Real> Vec3f;
inline const Vec3f operator*(const Real s, const Vec3f &r) { return r*s; }



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
  explicit Solver(
    const Real nu=0.04, const Real h=0.5, const Real density=1e3,
    const Vec3f g=Vec3f(0, -9.8, 0), const Real eta=0.01, const Real gamma=7.0)
    : _kernel(h), _nu(nu), _h(h), _d0(density),
      _g(g), _eta(eta), _gamma(gamma) {
    _dt = 0.006;
    _m0 = _d0*_h*_h*_h;
    _c = fabs(_g.y)/_eta;
    _p0 = _d0*_c*_c/_gamma;
  }



  // the size of f_width, f_height; each cell is sampled with 2x2 particles.
  void initScene(const int f_width, const int f_height, const int f_depth) {

    _pos.clear();
    // set wall for boundary
    _l = 0.5*_h;
    _r = static_cast<Real>(res_x) - 0.5*_h;
    _b = 0.5*_h;
    _t = static_cast<Real>(res_y) - 0.5*_h;
    _front = 0.5*_h;
    _back = static_cast<Real>(res_z) - 0.5*_h;

    // make sure for the other particle quantities
    _vel = vector<Vec3f>(_pos.size(), Vec3f(0, 0, 0));
    _acc = vector<Vec3f>(_pos.size(), Vec3f(0, 0, 0));
    _d   = vector<Real>(_pos.size(), 0);

  }
  int c = 0;

  void update() {
    cout << "." << flush;
    _acc = vector<Vec3f>(_pos.size(), Vec3f(0, 0, 0));
    applyBodyForce();
    updateVelocity();
    updatePosition();
    resolveCollision();
    //updateGridIsovalues();
    computeTriangles(c);
    c++;


  }

  tIndex particleCount() const { return _pos.size(); }
  const Vec3f& position(const tIndex i) const { return _pos[i]; }
  const float& color(const tIndex i) const { return _col[i]; }

  int resX() const { return _resX; }
  int resY() const { return _resY; }
  int resZ() const { return _resZ; }



private:


  void applyBodyForce() {
#pragma omp parallel for
    for(tIndex i=0; i<particleCount(); ++i) {
      _acc[i] += _g;
    }
  }



  void updateVelocity() {
#pragma omp parallel for
    for(tIndex i=0; i<particleCount(); ++i) {
      _vel[i] += _dt*_acc[i];   // simple forward Euler
    }
  }



  void updatePosition() {
#pragma omp parallel for
    for(tIndex i=0; i<particleCount(); ++i) {
      _pos[i] += _dt*_vel[i];
    }
  }

  // simple collision detection/resolution for each particle
  void resolveCollision() {
    vector<tIndex> need_res;
    for(tIndex i=0; i<particleCount(); ++i) {
      if(_pos[i].x<_l || _pos[i].y<_b || _pos[i].x>_r || _pos[i].y>_t || _pos[i].z>_back || _pos[i].z <_front)
        need_res.push_back(i);
    }

    for(
      vector<tIndex>::const_iterator it=need_res.begin();
      it<need_res.end();
      ++it) {
      const Vec3f p0 = _pos[*it];
      _pos[*it].x = clamp(_pos[*it].x, _l, _r);
      _pos[*it].y = clamp(_pos[*it].y, _b, _t);
      _pos[*it].z = clamp(_pos[*it].z, _front, _back);
      _vel[*it] = (_pos[*it] - p0)/_dt;
    }
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
  void computeTriangles(int c){
    //#pragma omp parallel for
    triangles.clear();
    for(tIndex i =0; i < gridcells.size(); i++) {
      Polygonise(gridcells[i],0, &triangles);
    }
    int n = triangles.size()/3;

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
  }


  }

  inline tIndex idx1d(const int i, const int j, const int k) { return i + j*resX() + k*resX()*resY(); }



  vector<Vec3f> triangles;
  // particle data
  vector<Vec3f> _pos;      // position
  vector<Vec3f> _vel;      // velocity
  vector<Vec3f> _acc;      // acceleration



  vector< vector<tIndex> > _pidxInGrid; // particle neighbor data

  vector<float> _col;      // particle color; just for visualization

  // simulation
  Real _dt;                     // time step

  int _resX, _resY, _resZ;             // background grid resolution

  // wall
  Real _l, _r, _b, _t, _front, _back;          // wall (boundary)

 // Coefficients

  Real _d0;                     // rest density
  Real _h;                      // particle spacing
  Vec3f  _g;                    // gravity

  Real _m0;                     // rest mass
  Real _p0;                     // EOS coefficient

  Real _omega;
  Real _eta;
  Real _c;                      // speed of sound
  Real _gamma;                  // EOS power factor
};

Solver solver(0.04, 0.5,  1e3, Vec3f(0, -9.8, 0), 0.01, 7.0);


int main(int argc, char **argv) {
  int nFrames = 240;
  solver.initScene(40, 32, 40, 16, 16, 8,8);
  for(int i = 0; i < nFrames;i++)
  solver.update();

  return EXIT_SUCCESS;
}
