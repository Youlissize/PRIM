// PROJECTIVE DYNAMICS

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <ctime>
#include <string>
#include <set>
#include <utility>
#include <chrono>
#include <omp.h>
//#include <execution>
#include "mesh.h"
#include "vector3.h"
#include "mySparseMatrix.h"
#include "fusingConstraint.h"
#include "scene.h"

using namespace std;
typedef float Real;
//typedef long int int;
typedef Eigen::Matrix<float,Eigen::Dynamic,1> FloatVector;
//typedef Eigen::Matrix<MyVec3,Eigen::Dynamic,1> Vec3Vector;
typedef Eigen::SparseMatrix<float> SparseMat;
//typedef Eigen::SparseMatrix<MyVec3> Vec3SparseMat;
typedef Eigen::DiagonalMatrix<float,Eigen::Dynamic> DiagMatrix;



// GLOBAL VARIABLES

// Objects
vector<Mesh> meshes;
Scene* scene = new Scene();
string objectFile;       // Mesh to import
string floorFile = "Meshes/floor.obj";

// simulation
int nFrames = 80;
Real h = 1.0f/24.0f;                     // time step
int solverIteration = 6;

// Coefficients
Vec3f  _g = Vec3f(0, -9.8f, 0);                    // gravity
float floorHeight = -1.0f; // for collision
float bounciness = 1.5f; //for collision
float friction = 0.5f; //for collision

// Variables
int N; //total number of vertices
FloatVector qn;  //q_n in paper
FloatVector qn1;  //q_n+1 in paper
FloatVector sn;       //s_n in paper
FloatVector velocity; //v_n in paper
SparseMat M; //diagonal matrix of mass
SparseMat M_inv; //diagonal matrix of 1/mass
FloatVector fext; //external forces
Eigen::SimplicialLDLT< Eigen::SparseMatrix<float> > _LHS_LDLT; //LinearSystem solver

//Constraints
vector<FixConstraint> fixConstraints = vector<FixConstraint>();
vector<StrainConstraint> strainConstraints = vector<StrainConstraint>();
vector<StretchConstraint> stretchConstraints = vector<StretchConstraint>();
vector<VolumeConstraint> volumeConstraints = vector<VolumeConstraint>();
vector<CollisionConstraint> collisionConstraints = vector<CollisionConstraint>();



// END GLOBAL VARIABLES


/* README
 -> Pour les vecteurs
 Eigen::VectorXd v( taille_Vecteur);

 -> Pour faire une Sparse Matrix :
Eigen::SparseMatrix<float> A(n_rows, n_columns);
MySparseMatrix A_mine( n_rows, n_columns );
A_mine(i,j) = valeur; [...]
A_mine.convertToEigenFormat(A);

 -> Pour résoudre un system lineaire de type AX = B
 Eigen::SparseMatrix< float > A;
 Eigen::VectorXd B;

 Eigen::SimplicialLDLT< Eigen::SparseMatrix<float> > _LHS_LDLT;
  _LHS_LDLT.analyzePattern( A );
  _LHS_LDLT.compute( A );
  Eigen::VectorXd X = _LHS_LDLT.solve( B );

  (à tester...)
*/




// LA CA DEVIENT INTERESSANT

class Solver {
public:
    //explicit Solver();

  void initScene() {

      //Import meshes
      meshes = vector<Mesh>();
      meshes.push_back(Mesh(objectFile,true,0.01f));
      //meshes.push_back(Mesh(floorFile,false));
      vector<Mesh*> meshesPointers = vector<Mesh*>();
      for (int i =0; i< meshes.size();++i){
        meshesPointers.push_back(&meshes[i]);
      }
      scene->setMeshes(meshesPointers);


      //Initialize variables
      N=0;
      for(auto& mesh:meshes){
        N+=mesh.meshVertices;
      }
      qn = FloatVector(3*N);
      velocity = FloatVector(3*N);
      fext = FloatVector(3*N);
      MySparseMatrix M_mine( 3*N, 3*N );
      MySparseMatrix M_inv_mine( 3*N, 3*N );

      int count = 0;
      for(auto& mesh:meshes){
        for(auto& v:mesh.vertices) {
          qn[3*count]= v.X.x;
          qn[3*count+1]= v.X.y;
          qn[3*count+2]= v.X.z;
          velocity[3*count]  = 0;
          velocity[3*count+1]  = 0;
          velocity[3*count+2]  = 0;
          fext[3*count] = 0;
          fext[3*count+1] = _g.y * (1.0f/v.w);
          fext[3*count+2] = 0;
          M_mine(3*count,3*count)=1.0f/v.w;
          M_mine(3*count+1,3*count+1)=1.0f/v.w;
          M_mine(3*count+2,3*count+2)=1.0f/v.w;
          M_inv_mine(3*count,3*count)=v.w;
          M_inv_mine(3*count+1,3*count+1)=v.w;
          M_inv_mine(3*count+2,3*count+2)=v.w;
          count ++;
        }
      }
      M_mine.convertToEigenFormat(M);
      M_inv_mine.convertToEigenFormat(M_inv);
      qn1 = FloatVector(3*N);
      sn = FloatVector(3*N);



      // Create constraints
      //Strain Constraints
      if(false) {
        float strainWeight = 10.0f;
        int offset = 0;
        for(auto& mesh : meshes){
          for(auto& t :mesh.triangles){
            strainConstraints.push_back( StrainConstraint(t.A+offset,t.B+offset,t.C+offset,qn,strainWeight) );
          }
          offset += mesh.meshVertices;
        }
      }

      //StretchConstraints
      if(false) {
        float stretchWeight = 10.0f;
        int offset = 0;
        for(auto& mesh : meshes){
          for(auto& e :mesh.edges){
            stretchConstraints.push_back( StretchConstraint(e.A+offset,e.B+offset,qn,stretchWeight) );
          }
          offset += mesh.meshVertices;
        }
      }

      // FixConstraints
      if(true){
        float fixWeight = 100000.0f;
        fixConstraints.push_back(FixConstraint(0,qn,fixWeight));
        //fixConstraints.push_back(FixConstraint(1,qn,fixWeight));
        /*
        for (int i =0; i < meshes[0].vertices.size(); ++i){
                if(meshes[0].vertices[i].X.y > 0.8)
                fixConstraints.push_back(FixConstraint(i,qn,fixWeight));
            }
        */
      }
      // VolumeConstraints
      if(true){
        float volumeWeight = 1.0f;
        int offset = 0;
        for(auto& mesh : meshes){
          if(mesh.isTetraedral){
          for(auto& t :mesh.tetras){
            volumeConstraints.push_back( VolumeConstraint(t.A+offset,t.B+offset,t.C+offset,t.D+ offset,qn,volumeWeight) );
          }
          offset += mesh.meshVertices;
        }
      }
      }

      //CollisionConstraint
      if(false){
        int count = 0;
        float collisionWeight = 100.0f;
        for(auto& mesh: meshes) {
          for(auto& v:mesh.vertices){
            collisionConstraints.push_back( CollisionConstraint(count, floorHeight, qn, collisionWeight ));
            count ++;
          }
        }
      }

      // Initial Deformation
      if(false){
        int count = 0;
        float deformation = 2.0f;
        for(auto& mesh: meshes) {
          for(auto& v:mesh.vertices){
            v.X *= deformation;
            qn[3*count]= v.X.x;
            qn[3*count+1]= v.X.y;
            qn[3*count+2]= v.X.z;
            count ++;
          }
        }
      }

      //Precompute system for global solving
      SparseMat leftSide = M / (h*h); //Matrix on the left side of equation (10)

      for (auto& c : strainConstraints) {
        if(c.AandBareIdentity){
          leftSide += c.w * c.S.transpose() * c.S;
        }
        else {
          leftSide += c.w * c.S.transpose() * c.A.transpose() * c.A * c.S;
        }
      }
      for (auto& c : stretchConstraints) {
        if(c.AandBareIdentity){
          leftSide += c.w * c.S.transpose() * c.S;
        }
        else {
          leftSide += c.w * c.S.transpose() * c.A.transpose() * c.A * c.S;
        }
      }

      for (auto& c : fixConstraints) {
        if(c.AandBareIdentity){
          leftSide += c.w * c.S.transpose() * c.S;
        }
        else {
          leftSide += c.w * c.S.transpose() * c.A.transpose() * c.A * c.S;
        }
      }
      for (auto& c : volumeConstraints) {
        if(c.AandBareIdentity){
          leftSide += c.w * c.S.transpose() * c.S;
        }
        else {
          leftSide += c.w * c.S.transpose() * c.A.transpose() * c.A * c.S;
        }
      }
      for (auto& c : collisionConstraints) {
        if(c.AandBareIdentity){
          leftSide += c.w * c.S.transpose() * c.S;
        }
        else {
          leftSide += c.w * c.S.transpose() * c.A.transpose() * c.A * c.S;
        }
      }
      _LHS_LDLT.analyzePattern( leftSide );
      _LHS_LDLT.compute( leftSide );

      writeOBJFile(0);
      cout<<"0 ";
  }


  int c = 1;
  void update() {
    cout << c << " " << flush;
    // Compute Sn
    sn = qn + velocity *h + M_inv.diagonal().asDiagonal()*fext * h*h;
    for(auto& fc : fixConstraints){ fc.fix(sn); }
    qn1 = sn;


    //Main solver loop
    SparseMat tmp = M / (h*h);
    for (int loopCount=0;loopCount<solverIteration;loopCount++){
      FloatVector rightSide = tmp.diagonal().asDiagonal()*sn; // right side of equation (10)

      //Local constraints solve (could be done in parralel)
      #pragma omp parallel
      for(int i =0; i< strainConstraints.size(); ++i) {
        StrainConstraint st = strainConstraints[i];
        st.project(qn1);
      }
      #pragma omp parallel
      for(int i =0; i<  stretchConstraints.size(); ++i) {
        StretchConstraint st = stretchConstraints[i];
        st.project(qn1);
      }
      #pragma omp parallel
      
      for(int l =0; l < volumeConstraints.size();++l) {
        VolumeConstraint vc = volumeConstraints[l];
        vc.project(qn1);
      }
      #pragma omp parallel
      for(int l =0; l < collisionConstraints.size(); ++l) {
        CollisionConstraint cc = collisionConstraints[l];
        cc.project(qn1);
      }
      // update rightSide
      for(auto& st : strainConstraints) {
          st.addProjection(rightSide);
      }
      for(auto& st : stretchConstraints) {
          st.addProjection(rightSide);
      }
      for(auto& fc : fixConstraints){ // Maybe useless
        fc.addProjection(rightSide);
      }
      for(auto& vc : volumeConstraints){ // Maybe useless
        vc.addProjection(rightSide);
      }
      for(auto& cc : collisionConstraints){ // Maybe useless
        cc.addProjection(rightSide);
      }
      //Global Solve
      qn1 = _LHS_LDLT.solve( rightSide );
    }


    for(auto& fc : fixConstraints) { fc.fix(qn1); }

    //Update velocity
    velocity = (qn1-qn) /h;
    for (auto& cc:collisionConstraints) {
      if(cc.inverseSpeed){
        velocity[3*cc.v+1] = abs(velocity[3*cc.v+1] * bounciness);
        velocity[3*cc.v] *= friction;
        velocity[3*cc.v+2] *= friction;
      }
    }
    qn = qn1;

    //Update Mesh and export
    updateMeshPos();
    writeOBJFile(c);
    c++;
  }


private:

  void updateMeshPos() {
    int count = 0;
    for(auto& mesh: meshes) {
      for(auto& v:mesh.vertices){
        v.X = Vec3f(qn[3*count],qn[3*count+1],qn[3*count+2]);
        count ++;
      }
    }
  }

  void writeOBJFile(int c){
    scene->exportToOBJ(c);
  }

};





int main(int argc, char *argv[]) {

  if(argc==1){
    cout<<"no input file"<<endl;
    return 0;
  }
  if(argc==2) {
      objectFile = argv[1];
  }
  else {
    cout<<"Too much arguments"<<endl;
    return 0;
  }


  cout<<"FUSING CONSTRAINT PROJECTION"<<endl;
  auto start = chrono::steady_clock::now();

  Solver solver;
  solver.initScene();

  //meshes[0].printVertexAndTrianglesAndEdges();
  auto initialisationTime = chrono::steady_clock::now();
  std::chrono::duration<double> initialisationSeconds = initialisationTime-start;

  for (int i = 1; i < nFrames; i++) {
      solver.update();
      scene->exportToOBJ(i);
  }
scene->writeMTL();
  cout << "State after " << nFrames << " frames : " << endl;
  //meshes[0].printVertexAndTrianglesAndEdges();

  auto end = chrono::steady_clock::now();
  chrono::duration<double> totalSeconds = end-start;
  chrono::duration<double> framesSeconds = end-initialisationTime;
  cout << "\nTotal number of vertices: "<<N<<endl;
  cout << "Initialisation time: " << initialisationSeconds.count() << "s\n";
  cout << "Total time: " << totalSeconds.count() << "s\n";
  cout << "Time for one frame: " << framesSeconds.count() / (float)nFrames << "s\n";



  return EXIT_SUCCESS;
}
