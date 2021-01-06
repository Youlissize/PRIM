#ifndef PROJCONSTRAINT
#define PROJCONSTRAINT

#include "eigen-3.3.9/Eigen/Sparse"
#include "eigen-3.3.9/Eigen/Dense"
#include "eigen-3.3.9/Eigen/SVD"
#include <vector>
#include <cmath>
#include <stdlib.h>
#include "mesh.h"
#include "mySparseMatrix.h"
//#include "myVec3.h"

typedef float Real;
//typedef long int int;
typedef Eigen::Matrix<float,Eigen::Dynamic,1> FloatVector;
//typedef Eigen::Matrix<MyVec3,Eigen::Dynamic,1> Vec3Vector;
typedef Eigen::SparseMatrix<float> SparseMat;
//typedef Eigen::SparseMatrix<MyVec3> Vec3SparseMat;
typedef Eigen::DiagonalMatrix<float,Eigen::Dynamic> DiagMatrix;
typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> FloatMatrix;

class FusingConstraint{

public:
  float w; //weigth of the constraint
  bool AandBareIdentity;
  SparseMat A;
  SparseMat B;
  SparseMat S;

  void project(FloatVector q) {};
  void addProjection(FloatVector& rs) {};

};

class FixConstraint : public FusingConstraint {
public:
  int v;
  Vec3f initialPos;

  FixConstraint(int _v, FloatVector q,float _w);
    void project(FloatVector q) ;
    void addProjection(FloatVector& qn);
    void fix(FloatVector& qn);
    };


class StrainConstraint: public FusingConstraint {
       public :
       int a,b,c;
       Vec3f v1_init,v2_init,v3_init,v1,v2,v3,x,y,norm;
       FloatMatrix Xg, Xf;
       float sMin, sMax;
       StrainConstraint(int a, int b, int c, FloatVector qn, float w);
       FloatMatrix computeX (Vec3f vec1, Vec3f vec2, Vec3f vec3);
       void project(FloatVector qn);
       void addProjection(FloatVector& rs) ;
};

class StretchConstraint: public FusingConstraint {
public:
  float initialLength;
  int v1,v2;
  Vec3f dV1,dV2,q1,q2;
  StretchConstraint (int _v1, int _v2, FloatVector qn,float _w);

  void project(FloatVector q) ;
  void addProjection(FloatVector& rs);



  };




  #endif
