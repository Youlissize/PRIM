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

  Edge e;
  float initialLength;

  void project() {

    Vec3f dpA;

    // to continue
    
    cout<<"Resolved!"<<endl;
  }
};









#endif
