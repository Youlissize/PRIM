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

  Eigen::VectorXd project(Eigen::VectorXd q) {};
};


class StretchConstraint: FusingConstraint {
public:
  float initialLength;


  Eigen::VectorXd project(Eigen::VectorXd q) {
    Eigen::VectorXd result = Eigen::VectorXd();
    return result;
  }

  //constructor
  StretchConstraint (Mesh* mesh,Edge e) {
    AandBareIdentity = true;


    this-> initialLength = 0;
  }





};




















#endif
