#ifndef PROJCONSTRAINT
#define PROJCONSTRAINT

#include "eigen-3.3.8/Eigen/Sparse"
#include "eigen-3.3.8/Eigen/Dense"
#include "eigen-3.3.8/Eigen/SVD"
#include "myVec3.cpp"

typedef float Real;
typedef long int tIndex;
typedef Eigen::Matrix<float,Eigen::Dynamic,1> floatVector;
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

  void project(Vec3Vector q) {};
  void addProjection(Vec3Vector& rs) {};

};



class StretchConstraint: public FusingConstraint {
public:
  float initialLength;
  tIndex v1,v2;
  MyVec3 dV1,dV2;


  void project(Vec3Vector q)  {

    float d = length(q[v1]-q[v2]);
    if(d>0.000001){
      dV1 = -0.5f * (d-initialLength) * (q[v1] - q[v2]) / d;
      dV2 = 0.5f * (d-initialLength) * (q[v1] - q[v2]) / d;
    }

  }

  void addProjection(Vec3Vector& rs) {
    rs[v1] += w*dV1;
    rs[v2] += w*dV2;
  };




  //constructor
  StretchConstraint (tIndex _v1, tIndex _v2, Vec3Vector qn,float _w) {
    w=_w;
    v1=_v1;
    v2=_v2;
    tIndex N = qn.rows();

    AandBareIdentity = true;
    A=SparseMat();
    B=SparseMat();
    S = SparseMat();
    MySparseMatrix MyS = MySparseMatrix(N,N);
    MyS(v1,v1)=1.f;
    MyS(v2,v2)=1.f;
    MyS.convertToEigenFormat(S);


    this-> initialLength = length(qn[v1]-qn[v2]);
  }





};




















#endif
