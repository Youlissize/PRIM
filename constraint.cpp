#ifndef CONSTRAINT
#define CONSTRAINT
#include "vector3.cpp"
#include "common.h"
#include <math.h>
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
  Vertex* p1; // p1 and p2 are centrale vertices
  Vertex* p2;
  Vertex* p3;
  Vertex* p4;
  float phi0;

  void project() {
    Vec3f e2 = p2->P - p1->P;
    Vec3f e3 = p3->P - p1->P;
    Vec3f e4 = p4->P - p1->P;
    if(e2.crossProduct(e3).lengthSquare() + e2.crossProduct(e4).lengthSquare() > 0.00001) {
      Vec3f n1 = (e2.crossProduct(e3)).normalize();
      Vec3f n2 = (e2.crossProduct(e4)).normalize();
      float d = n1.dotProduct(n2);

      Vec3f q1,q2,q3,q4;
      q3 = (e2.crossProduct(n2)+d*n1.crossProduct(e2)) / e2.crossProduct(e3).length();
      q4 = (e2.crossProduct(n1)+d*n2.crossProduct(e2)) / e2.crossProduct(e4).length();
      q2 = - ( (e3.crossProduct(n2)+d*n1.crossProduct(e3)) / e2.crossProduct(e3).length()  )
              - ( (e4.crossProduct(n1)+d*n2.crossProduct(e4)) / e2.crossProduct(e4).length() );
      q1 = -q2 - q3 - q4;


      float sumSquare = q1.lengthSquare()+q2.lengthSquare()+q3.lengthSquare()+q4.lengthSquare();
      float sumWeight = p1->w+p2->w+p3->w+p4->w ;
      if(sumSquare>0.000001 && sumWeight>0.000001 && abs(d)<1) {
        //cout<<"Q : "<<q1<<" / "<<q2<<" / "<<q3<<" / "<<q4<<" / "<<endl;
        //cout<<" d = "<<d<<endl;
        //cout<<sqrt(1-d*d)<<"  //  "<<(acos(d)-phi0)<<endl;

        float L = -(4.f/ sumWeight ) * sqrt(1-d*d)* (acos(d)-phi0) / sumSquare;

        //cout<<"L = "<<L<<endl;

        p1->P += p1->w * L * q1;
        p2->P += p2->w * L * q2;
        p3->P += p3->w * L * q3;
        p4->P += p4->w * L * q4;
      }
    }

  }

  //constructor
  BendConstraint(Vertex* v1,Vertex* v2,Vertex* v3,Vertex* v4){
    p1=v1;
    p2=v2;
    p3=v3;
    p4=v4;

    // compute phi0
    Vec3f e1 = p2->X - p1->X;
    Vec3f e2 = p3->X - p1->X;
    Vec3f e3 = p4->X - p1->X;
    Vec3f n1 = (e1.crossProduct(e2)).normalize();
    Vec3f n2 = (e1.crossProduct(e3)).normalize();
    float d = n1.dotProduct(n2);
    if(d<-0.999){
      phi0 = 3.141592654;
    }
    else if(d>0.999){
      phi0 = 0;
    }
    else { phi0 = acos(d); }


  }





};







#endif
