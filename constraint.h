#ifndef CONSTRAINT
#define CONSTRAINT
#include "vector3.h"
#include <math.h>
#include "mesh.h"

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
 void project();
LengthConstraint(Vertex* vA, Vertex* vB, float k_);
};

class FixConstraint : public Constraint {
public:

  Vertex * V;
  Vec3f initialPos;
  FixConstraint(Vertex* vertex);
 void project();

  };

class BendConstraint : public Constraint {
  public:
    Vertex* p1; // p1 and p2 are centrale vertices
    Vertex* p2;
    Vertex* p3;
    Vertex* p4;
    float phi0;
    float kb;
     void project();

    BendConstraint(Vertex* v1,Vertex* v2,Vertex* v3,Vertex* v4, float k_bending);
};

class CollisionConstraint : public Constraint{
public:
  float k_bouncing;
  Vertex* V;
  Vec3f normal;
  Vec3f collisionPoint;
  Vec3f originalDirection;
  Vec3f newDirection;
  void project();

  CollisionConstraint(Vertex* vertex,Vec3f collisionPoint, Vertex t1, Vertex t2, Vertex t3, float k_bouncing );
};
#endif
