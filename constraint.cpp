#ifndef CONSTRAINT
#define CONSTRAINT
#include "vector3.cpp"
#include "common.h"
using namespace std;



class Constraint{
public:
  void project() {};
};

class LengthConstraint : public Constraint {
public:

  Vertex * A;
  Vertex * B;
  float initialLength;
  float k;

  void project() {
    float d = (A->P - B->P).length();
    if((A->w + B->w)>0){
    Vec3f dpA = -(A->w / (A->w + B->w)) * (d-initialLength) * (A->P - B->P) / d;
    Vec3f dpB = (B->w / (A->w + B->w)) * (d-initialLength) * (A->P - B->P) / d;

    A->P += k*dpA;
    B->P += k*dpB;}


    }

  //constructor
  LengthConstraint(Vertex* vA, Vertex* vB, float k_){
    A = vA;
    B = vB;
    k = k_;
    initialLength = (A->X - B->X).length();

  }
};

class FixConstraint : public Constraint {
public:

  Vertex * V;
  Vec3f initialPos;

  void project() {
    V->P = initialPos;
  }

  //constructor
  FixConstraint(Vertex* vertex) {
    V = vertex;
    V->w = 0;
    initialPos = V->X;
  }
};

class BendConstraint : public Constraint {
public:
  Vertex* p1,p2,p3,p4; // p1 and p2 are centrale vertices
  float phi0;

  void project() {

  }

  //constructor
  BendConstraint(Vertex* v1,Vertex* v2,Vertex* v3,Vertex* v4){
    p1=v1;
    p2=v2;
    p3=v3;
    p4=v4;

    // compute phi0



  }





}







#endif
