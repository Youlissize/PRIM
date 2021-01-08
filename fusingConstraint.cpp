#include "fusingConstraint.h"

  //constructor
  FixConstraint::FixConstraint(int _v, FloatVector q,float _w) {
    w =_w;
    v=_v;
    initialPos = Vec3f(q[3*v],q[3*v+1],q[3*v+2]);
    AandBareIdentity = true;
    S = SparseMat();
    int N = q.rows()/3;
    MySparseMatrix MyS = MySparseMatrix(3*N,3*N);
    MyS(3*v,3*v)=1.f;
    MyS(3*v+1,3*v+1)=1.f;
    MyS(3*v+2,3*v+2)=1.f;
    MyS.convertToEigenFormat(S);

  }
  void FixConstraint::project(FloatVector q) {};

  void FixConstraint::addProjection(FloatVector& qn) {
    qn[3*v]+=w*initialPos.x;
    qn[3*v+1]+=w*initialPos.y;
    qn[3*v+2]+=w*initialPos.z;
  }

  void FixConstraint::fix(FloatVector& qn) {
    qn[3*v]=initialPos.x;
    qn[3*v+1]=initialPos.y;
    qn[3*v+2]=initialPos.z;
  }





   //Constructor

    StrainConstraint::StrainConstraint(int a, int b, int c, FloatVector qn, float w){
    int N = qn.rows()/3;
    v1_init = Vec3f(qn[3*a],qn[3*a+1], qn[3*a+2]);
    v2_init = Vec3f(qn[3*b],qn[3*b+1], qn[3*b+2]);
    v3_init = Vec3f(qn[3*c],qn[3*c+1], qn[3*c+2]);
    this->a = a;
    this->b = b;
    this->c = c;
    this->w = w;
    Xg = computeX(v1_init,v2_init,v3_init);
    AandBareIdentity = false;
    A=SparseMat();
    B=SparseMat();
    MySparseMatrix MyA = MySparseMatrix(3*N,3*N);
    MyA(3*a,3*a)=2.0f/3.0f;
    MyA(3*a+1,3*a+1)=2.0f/3.0f;
    MyA(3*a+2,3*a+2)=2.0f/3.0f;
    MyA(3*b,3*b)=2.0f/3.0f;
    MyA(3*b+1,3*b+1)=2.0f/3.0f;
    MyA(3*b+2,3*b+2)=2.0f/3.0f;
    MyA(3*c,3*c)=2.0f/3.0f;
    MyA(3*c+1,3*c+1)=2.0f/3.0f;
    MyA(3*c+2,3*c+2)=2.0f/3.0f;

    MyA(3*a,3*b)=-1.0f/3.0f;
    MyA(3*a+1,3*b+1)=-1.0f/3.0f;
    MyA(3*a+2,3*b+2)=-1.0f/3.0f;
    MyA(3*a,3*c)=-1.0f/3.0f;
    MyA(3*a+1,3*c+1)=-1.0f/3.0f;
    MyA(3*a+2,3*c+2)=-1.0f/3.0f;

    MyA(3*b,3*a)=-1.0f/3.0f;
    MyA(3*b+1,3*a+1)=-1.0f/3.0f;
    MyA(3*b+2,3*a+2)=-1.0f/3.0f;
    MyA(3*b,3*c)=-1.0f/3.0f;
    MyA(3*b+1,3*c+1)=-1.0f/3.0f;
    MyA(3*b+2,3*c+2)=-1.0f/3.0f;

    MyA(3*c,3*b)=-1.0f/3.0f;
    MyA(3*c+1,3*b+1)=-1.0f/3.0f;
    MyA(3*c+2,3*b+2)=-1.0f/3.0f;
    MyA(3*c,3*a)=-1.0f/3.0f;
    MyA(3*c+1,3*a+1)=-1.0f/3.0f;
    MyA(3*c+2,3*a+2)=-1.0f/3.0f;

    MyA.convertToEigenFormat(A);
    B=A;

    S = SparseMat();
    MySparseMatrix MyS = MySparseMatrix(3*N,3*N);
    MyS(3*a,3*a)=1.f;
    MyS(3*a+1,3*a+1)=1.f;
    MyS(3*a+2,3*a+2)=1.f;
    MyS(3*b,3*b)=1.f;
    MyS(3*b+1,3*b+1)=1.f;
    MyS(3*b+2,3*b+2)=1.f;
    MyS(3*c,3*c)=1.f;
    MyS(3*c+1,3*c+1)=1.f;
    MyS(3*c+2,3*c+2)=1.f;
    MyS.convertToEigenFormat(S);
    sMin = 0.95;
    sMax = 1.05;

   }

   FloatMatrix StrainConstraint::computeX (Vec3f vec1, Vec3f vec2, Vec3f vec3)
    {
      norm = (vec3-vec1).crossProduct(vec3-vec2).normalize();
      x = (vec2-vec1).normalize();
      y = x.crossProduct(norm).normalize();
      FloatMatrix X = FloatMatrix(2,2);
      X(0,0) = (vec1-vec2).length();
      X(1,0) = 0.f;
      X(0,1) = (vec3-vec1).dotProduct(x);
      X(1,1) = (vec3-vec1).dotProduct(y);

      return X;
    }

    void StrainConstraint::project(FloatVector qn){
    v1 = Vec3f(qn[3*a],qn[3*a+1], qn[3*a+2]);
    v2 = Vec3f(qn[3*b],qn[3*b+1], qn[3*b+2]);
    v3 = Vec3f(qn[3*c],qn[3*c+1], qn[3*c+2]);
    Vec3f Bary = (v1+v2+v3)/3.0f;
    Xf = computeX(v1,v2,v3);
    FloatMatrix m = FloatMatrix(2,2);
    m = Xf*Xg.inverse();
    Eigen::JacobiSVD< FloatMatrix > svdStruct = m.jacobiSvd( Eigen::ComputeFullU | Eigen::ComputeFullV );
    FloatVector sigma = svdStruct.singularValues();
    //Clamping values
    for (int i =0; i <2; ++i){
        if(sigma[i] < sMin) sigma[i] = sMin;
        if(sigma[i] > sMax) sigma[i] = sMax;
        }
    FloatMatrix t = svdStruct.matrixU()*sigma.asDiagonal()*svdStruct.matrixV().transpose();

    t = t*Xg; // ******************************************************************************               Multiplication logique mais douteuse, � tester sans !
    v2 = v1 + x*t(0,0) +y*t(1,0);
    v3 = v1 + x*t(0,1) +y*t(1,1);
    Vec3f BaryProj = (v1+v2+v3)/3.0f;
    v1+=(Bary-BaryProj);
    v2+=(Bary-BaryProj);
    v3+=(Bary-BaryProj);

  /*  SparseMat Col = SparseMat();
    MySparseMatrix MyCol = MySparseMatrix(qn.size(),1);
    MyCol(3*a,0) = v1.x;
    MyCol(3*a+1,0) = v1.y;
    MyCol(3*a+2,0) = v1.z;
    MyCol(3*b,0) = v2.x;
    MyCol(3*b+1,0) = v2.y;
    MyCol(3*b+2,0) = v2.z;
    MyCol(3*c,0) = v3.x;
    MyCol(3*c+1,0) = v3.y;
    MyCol(3*c+2,0) = v3.z;
    MyCol.convertToEigenFormat(Col);
    SparseMat newCol = A.transpose() * B * Col;
    v1 = Vec3f(newCol.coeffRef(3*a,0),newCol.coeffRef(3*a+1,0),newCol.coeffRef(3*a+2,0));
    v2 = Vec3f(newCol.coeffRef(3*b,0),newCol.coeffRef(3*b+1,0),newCol.coeffRef(3*b+2,0));
    v3 = Vec3f(newCol.coeffRef(3*c,0),newCol.coeffRef(3*c+1,0),newCol.coeffRef(3*c+2,0)); */


    }
    void StrainConstraint::addProjection(FloatVector& rs) {
      if(AandBareIdentity==false){
          FloatVector pi = FloatVector(rs.rows());
          pi[3*a] = w*v1.x;
          pi[3*a+1] = w*v1.y;
          pi[3*a+2] = w*v1.z;
          pi[3*b] = w*v2.x;
          pi[3*b+1] = w*v2.y;
          pi[3*b+2] = w*v2.z;
          pi[3*c] = w*v3.x;
          pi[3*c+1] = w*v3.y;
          pi[3*c+2] = w*v3.z;
          pi = S.transpose()*A.transpose()*B*pi;
          rs += pi;
    }
    else {
      rs[3*a] += w*v1.x;
      rs[3*a+1] += w*v1.y;
      rs[3*a+2] += w*v1.z;
      rs[3*b] += w*v2.x;
      rs[3*b+1] += w*v2.y;
      rs[3*b+2] += w*v2.z;
      rs[3*c] += w*v3.x;
      rs[3*c+1] += w*v3.y;
      rs[3*c+2] += w*v3.z;
    }
  }





  void StretchConstraint::project(FloatVector q)  {
    q1 = Vec3f(q[3*v1],q[3*v1+1],q[3*v1+2]);
    q2 = Vec3f(q[3*v2],q[3*v2+1],q[3*v2+2]);

    float d = (q1-q2).length();
    if(d>0.000001){
      dV1 = (q1 - 0.5f * (d-initialLength) * (q1 - q2) / d);
      dV2 = (q2 + 0.5f * (d-initialLength) * (q1 - q2) / d);
    }

  }

  void StretchConstraint::addProjection(FloatVector& rs) {
    if (AandBareIdentity==false) {
      FloatVector pi = FloatVector(rs.rows());
      pi[3*v1] = w*dV1.x;
      pi[3*v1+1] = w*dV1.y;
      pi[3*v1+2] = w*dV1.z;
      pi[3*v2] = w*dV2.x;
      pi[3*v2+1] = w*dV2.y;
      pi[3*v2+2] = w*dV2.z;
      pi = S.transpose()*A.transpose()*B*pi;
      rs += pi;
    }
    else {
      rs[3*v1] += w*dV1.x;
      rs[3*v1+1] += w*dV1.y;
      rs[3*v1+2] += w*dV1.z;
      rs[3*v2] += w*dV2.x;
      rs[3*v2+1] += w*dV2.y;
      rs[3*v2+2] += w*dV2.z;
    }

  };




  //constructor
  StretchConstraint::StretchConstraint (int _v1, int _v2, FloatVector qn,float _w) {
    w=_w;
    v1=_v1;
    v2=_v2;
    int N = qn.rows()/3;
    q1 = Vec3f(qn[3*_v1],qn[3*_v1+1],qn[3*_v1+2]);
    q2 = Vec3f(qn[3*_v2],qn[3*_v2+1],qn[3*_v2+2]);
    AandBareIdentity = false;
    A=SparseMat();
    B=SparseMat();
    MySparseMatrix MyA = MySparseMatrix(3*N,3*N);

    MyA(3*v1,3*v1)=1.0f/2.0f;
    MyA(3*v1+1,3*v1+1)=1.0f/2.0f;
    MyA(3*v1+2,3*v1+2)=1.0f/2.0f;
    MyA(3*v2,3*v2)=1.0f/2.0f;
    MyA(3*v2+1,3*v2+1)=1.0f/2.0f;
    MyA(3*v2+2,3*v2+2)=1.0f/2.0f;

    MyA(3*v1,3*v2)=-1.0f/2.0f;
    MyA(3*v1+1,3*v2+1)=-1.0f/2.0f;
    MyA(3*v1+2,3*v2+2)=-1.0f/2.0f;
    MyA(3*v2,3*v1)=-1.0f/2.0f;
    MyA(3*v2+1,3*v1+1)=-1.0f/2.0f;
    MyA(3*v2+2,3*v1+2)=-1.0f/2.0f;
    MyA.convertToEigenFormat(A);
    B=A;

    S = SparseMat();
    MySparseMatrix MyS = MySparseMatrix(3*N,3*N);
    MyS(3*v1,3*v1)=1.f;
    MyS(3*v1+1,3*v1+1)=1.f;
    MyS(3*v1+2,3*v1+2)=1.f;
    MyS(3*v2,3*v2)=1.f;
    MyS(3*v2+1,3*v2+1)=1.f;
    MyS(3*v2+2,3*v2+2)=1.f;

    MyS.convertToEigenFormat(S);


    this-> initialLength = (q1-q2).length();
  }

VolumeConstraint::VolumeConstraint(int a, int b, int c,int d, FloatVector qn, float w){
  int N = qn.rows()/3;
  v1_init = Vec3f(qn[3*a],qn[3*a+1], qn[3*a+2]);
  v2_init = Vec3f(qn[3*b],qn[3*b+1], qn[3*b+2]);
  v3_init = Vec3f(qn[3*c],qn[3*c+1], qn[3*c+2]);
  v4_init = Vec3f(qn[3*d],qn[3*d+1], qn[3*d+2]);

  this->a = a;
  this->b = b;
  this->c = c;
  this->d = d;
  this->w = w;

  AandBareIdentity = true;

  Xg = computeX  (v1_init,v2_init,v3_init,v4_init);

  S = SparseMat();
  MySparseMatrix MyS = MySparseMatrix(3*N,3*N);
  MyS(3*a,3*a)=1.f;
  MyS(3*a+1,3*a+1)=1.f;
  MyS(3*a+2,3*a+2)=1.f;
  MyS(3*b,3*b)=1.f;
  MyS(3*b+1,3*b+1)=1.f;
  MyS(3*b+2,3*b+2)=1.f;
  MyS(3*c,3*c)=1.f;
  MyS(3*c+1,3*c+1)=1.f;
  MyS(3*c+2,3*c+2)=1.f;
  MyS(3*d,3*d)=1.f;
  MyS(3*d+1,3*d+1)=1.f;
  MyS(3*d+2,3*d+2)=1.f;
  MyS.convertToEigenFormat(S);
  sMin = 0.95;
  sMax = 1.05;

}
FloatMatrix VolumeConstraint::computeX (Vec3f vec1, Vec3f vec2, Vec3f vec3, Vec3f vec4){
  FloatMatrix X = FloatMatrix(3,3);
  X(0,0) = (vec1-vec4).x;
  X(1,0) = (vec1-vec4).y;
  X(2,0) = (vec1-vec4).z;
  X(0,1) = (vec2-vec4).x;
  X(1,1) = (vec2-vec4).y;
  X(2,1) = (vec2-vec4).z;
  X(0,2) = (vec3-vec4).x;
  X(1,2) = (vec3-vec4).y;
  X(2,2) = (vec3-vec4).z;
  return X;
}

void VolumeConstraint::project(FloatVector qn){
  v1 = Vec3f(qn[3*a],qn[3*a+1], qn[3*a+2]);
  v2 = Vec3f(qn[3*b],qn[3*b+1], qn[3*b+2]);
  v3 = Vec3f(qn[3*c],qn[3*c+1], qn[3*c+2]);
  v4 = Vec3f(qn[3*d],qn[3*d+1], qn[3*d+2]);
  Vec3f Bary = (v1+v2+v3+v4)/4.0f;
  Xf = computeX(v1,v2,v3,v4);
  FloatMatrix m = FloatMatrix(3,3);
  m = Xf*Xg.inverse();

  Eigen::JacobiSVD< FloatMatrix > svdStruct = m.jacobiSvd( Eigen::ComputeFullU | Eigen::ComputeFullV );
  FloatVector sigma = svdStruct.singularValues();

  //Clamping values
  for (int i =0; i <3; ++i){
    //cout << sigma[i] << endl;
      if(sigma[i] < sMin) sigma[i] = sMin;
      if(sigma[i] > sMax) sigma[i] = sMax;
      }
      if (m.determinant() <0 ){
        sigma *= -1;
      }
  FloatMatrix t = svdStruct.matrixU()*sigma.asDiagonal()*svdStruct.matrixV().transpose();

  t = t*Xg; // ******************************************************************************               Multiplication logique mais douteuse, � tester sans !

  v1 = v4 + Vec3f(t(0,0),t(1,0),t(2,0));
  v2 = v4 + Vec3f(t(0,1),t(1,0),t(2,1));
  v3 = v4 + Vec3f(t(0,2),t(1,2),t(2,2));


  Vec3f BaryProj = (v1+v2+v3+v4)/4.0f;
  v1+=(Bary-BaryProj);
  v2+=(Bary-BaryProj);
  v3+=(Bary-BaryProj);
  v4+=(Bary-BaryProj);

}


void VolumeConstraint::addProjection(FloatVector& rs){
  if(AandBareIdentity==false){
      FloatVector pi = FloatVector(rs.rows());
      pi[3*a] = w*v1.x;
      pi[3*a+1] = w*v1.y;
      pi[3*a+2] = w*v1.z;
      pi[3*b] = w*v2.x;
      pi[3*b+1] = w*v2.y;
      pi[3*b+2] = w*v2.z;
      pi[3*c] = w*v3.x;
      pi[3*c+1] = w*v3.y;
      pi[3*c+2] = w*v3.z;
      pi[3*d] = w*v4.x;
      pi[3*d+1] = w*v4.y;
      pi[3*d+2] = w*v4.z;
      pi = S.transpose()*A.transpose()*B*pi;
      rs += pi;
  }
  else {
  rs[3*a] += w*v1.x;
  rs[3*a+1] += w*v1.y;
  rs[3*a+2] += w*v1.z;
  rs[3*b] += w*v2.x;
  rs[3*b+1] += w*v2.y;
  rs[3*b+2] += w*v2.z;
  rs[3*c] += w*v3.x;
  rs[3*c+1] += w*v3.y;
  rs[3*c+2] += w*v3.z;
  rs[3*d] += w*v4.x;
  rs[3*d+1] += w*v4.y;
  rs[3*d+2] += w*v4.z;
  }

}
