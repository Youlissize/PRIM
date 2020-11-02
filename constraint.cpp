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
    if((A->w + B->w)>0 && d>0.0000001){
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
  float kb;

  void project() {

    float epsilon = 0.000001;

    Vec3f e2 = p2->P - p1->P;
    Vec3f e3 = p3->P - p1->P;
    Vec3f e4 = p4->P - p1->P;
    float length23 = e2.crossProduct(e3).length();
    float length24 = e2.crossProduct(e4).length();

    if( length23> epsilon &&  length24> epsilon) { //avoid division by 0
      Vec3f n1 = (e2.crossProduct(e3)).normalize();
      Vec3f n2 = (e2.crossProduct(e4)).normalize();
      float d = n1.dotProduct(n2);

      Vec3f q1,q2,q3,q4;
      q3 = (e2.crossProduct(n2)+d*n1.crossProduct(e2)) / length23;
      q4 = (e2.crossProduct(n1)+d*n2.crossProduct(e2)) / length24;
      q2 = - ( (e3.crossProduct(n2)+d*n1.crossProduct(e3)) / length23  )
              - ( (e4.crossProduct(n1)+d*n2.crossProduct(e4)) / length24 );
      q1 = -q2 - q3 - q4;


      float sumSquare = q1.lengthSquare()+q2.lengthSquare()+q3.lengthSquare()+q4.lengthSquare();
      float sumWeight = p1->w+p2->w+p3->w+p4->w ;
      if(sumSquare>epsilon && sumWeight>epsilon && abs(d)<1) {  //avoid division by 0;
        //cout<<"Q : "<<q1<<" / "<<q2<<" / "<<q3<<" / "<<q4<<" / "<<endl;
        //cout<<" d = "<<d<<endl;
        //cout<<sqrt(1-d*d)<<"  //  "<<(acos(d)-phi0)<<endl;

        float L = -(4.f/ sumWeight ) * sqrt(1-d*d)* (acos(d)-phi0) / sumSquare;


        if(L>100){
      /*    cout<<"Q : "<<q1<<" / "<<q2<<" / "<<q3<<" / "<<q4<<" / "<<endl;
          cout<<length23<<" -- "<<length24<<endl;
          cout<<" d = "<<d<<endl;
          cout<<sqrt(1-d*d)<<"  //  "<<(acos(d)-phi0)<<endl;
          cout<<"L = "<<L<<endl;*/
        }

        p1->P += kb * p1->w * L * q1;
        p2->P += kb * p2->w * L * q2;
        p3->P += kb * p3->w * L * q3;
        p4->P += kb * p4->w * L * q4;
      }
    }

  }

  //constructor
  BendConstraint(Vertex* v1,Vertex* v2,Vertex* v3,Vertex* v4, float k_bending){
    p1=v1;
    p2=v2;
    p3=v3;
    p4=v4;
    kb = k_bending;
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
class CollisionConstraint : public Constraint{
public:
  float k_bouncing;
  Vertex* V;
  Vec3f normal;
  Vec3f collisionPoint;
  Vec3f originalDirection;
  Vec3f newDirection;

  void project() {
    V->P = V->X + k_bouncing*(collisionPoint - V->P).length()*newDirection;


  }

  //constructor
  CollisionConstraint(Vertex* vertex,Vec3f collisionPoint, Vertex t1, Vertex t2, Vertex t3, float k_bouncing ) {
    V = vertex;
    normal = (t2.X-t1.X).crossProduct(t3.X-t2.X).normalize();
    this->collisionPoint = collisionPoint;
    originalDirection = (V->P - V->X).normalize();
    newDirection = (originalDirection + 2*normal).normalize();
  }

};



#endif
