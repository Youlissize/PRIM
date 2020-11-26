#ifndef PROJCONSTRAINT
#define PROJCONSTRAINT

#include "eigen-3.3.8/Eigen/Sparse"
#include "eigen-3.3.8/Eigen/Dense"
#include "eigen-3.3.8/Eigen/SVD"
#include "myVec3.cpp"

typedef float Real;
typedef long int tIndex;
typedef Eigen::Matrix<float,Eigen::Dynamic,1> FloatVector;
typedef Eigen::Matrix<MyVec3,Eigen::Dynamic,1> Vec3Vector;
typedef Eigen::SparseMatrix<float> SparseMat;
typedef Eigen::SparseMatrix<MyVec3> Vec3SparseMat;
typedef Eigen::DiagonalMatrix<float,Eigen::Dynamic> DiagMatrix;


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



class StretchConstraint: public FusingConstraint {
public:
  float initialLength;
  tIndex v1,v2;
  Vec3f dV1,dV2,q1,q2;



  void project(FloatVector q)  {
    q1 = Vec3f(q[3*v1],q[3*v1+1],q[3*v1+2]);
    q2 = Vec3f(q[3*v2],q[3*v2+1],q[3*v2+2]);

    float d = length(q1-q2);
    if(d>0.000001){
      dV1 = 1.f/w*q1-0.5f * (d-initialLength) * (q1 - q2) / d;
      dV2 = 1.f/w*q2+0.5f * (d-initialLength) * (q1 - q2) / d;
    }

  }

  void addProjection(FloatVector& rs) {
    rs[3*v1] += w*dV1.x;
    rs[3*v1+1] += w*dV1.y;
    rs[3*v1+2] += w*dV1.z;
    rs[3*v2] += w*dV2.x;
    rs[3*v2+1] += w*dV2.y;
    rs[3*v2+2] += w*dV2.z;
  };




  //constructor
  StretchConstraint (tIndex _v1, tIndex _v2, FloatVector qn,float _w) {
    w=_w;
    v1=_v1;
    v2=_v2;
    tIndex N = qn.rows()/3;
    q1 = Vec3f(qn[3*_v1],qn[3*_v1+1],qn[3*_v1+2]);
    q2 = Vec3f(qn[3*_v2],qn[3*_v2+1],qn[3*_v2+2]);
    AandBareIdentity = true;
    A=SparseMat();
    B=SparseMat();
    S = SparseMat();
    MySparseMatrix MyS = MySparseMatrix(3*N,3*N);
    MyS(3*v1,3*v1)=1.f;
    MyS(3*v1+1,3*v1+1)=1.f;
    MyS(3*v1+2,3*v1+2)=1.f;
    MyS(3*v2,3*v2)=1.f;
    MyS(3*v2+1,3*v2+1)=1.f;
    MyS(3*v2+2,3*v2+2)=1.f;
    MyS.convertToEigenFormat(S);


    this-> initialLength = length(q1-q2);
  }





};




#endif
