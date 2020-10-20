#ifndef CONSTRAINT
#define CONSTRAINT
#include "vector3.cpp"
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

  void project() {
    float d = (A->P - B->P).length();

    Vec3f dpA = -(A->w / (A->w + B->w)) * (d-initialLength) * (A->P - B->P) / d;
    Vec3f dpB = (B->w / (A->w + B->w)) * (d-initialLength) * (A->P - B->P) / d;

    A->P += dpA;
    B->P += dpB;

    cout<<"Resolved!"<<endl;

    //constructor
    LengthConstraint(Vertex* vA, Vertex* vB){
      A = vA;
      B = vB;
      initialLength = (A->X - B->X).length();
    }
  }
};

class FixConstraint : public Constraint {
public:

  Vertex * V;
  Vec3f initialPos;

  void project() {
    V->P = initialPos;

    cout<<"Resolved!"<<endl;
  }

  //constructor
  FixConstraint(Vertex* vertex) {
    V = vertex;
    V->w = 0;
    initialPos = V->X;
  }
};







#endif
