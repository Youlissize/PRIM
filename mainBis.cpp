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
#include "mesh.cpp"
#include "vector3.cpp"
#include "mySparseMatrix.cpp"
#include "fusingConstraint.cpp"
#include "Scene.cpp"
#include "myVec3.cpp"

using namespace std;
typedef float Real;
//typedef long int int;
typedef Eigen::Matrix<float,Eigen::Dynamic,1> FloatVector;
typedef Eigen::Matrix<MyVec3,Eigen::Dynamic,1> Vec3Vector;
typedef Eigen::SparseMatrix<float> SparseMat;
typedef Eigen::SparseMatrix<MyVec3> Vec3SparseMat;
typedef Eigen::DiagonalMatrix<float,Eigen::Dynamic> DiagMatrix;



// GLOBAL VARIABLES

// Objects
vector<Mesh> meshes;
Scene* scene = new Scene();
string objectFile = "Meshes/sphere.obj";       // Mesh to import
string floorFile = "Meshes/floor.obj";

// simulation
int nFrames = 100;
Real h = 0.05;                     // time step
int solverIteration = 4;

// Coefficients
Vec3f  _g = Vec3f(0, -9.8, 0);                    // gravity

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
      if(true) {
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
        float stretchWeight = 0.5f;
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
        float fixWeight = 1.0f;
        fixConstraints.push_back(FixConstraint(2,qn,fixWeight));
        //fixConstraints.push_back(FixConstraint(1,qn,fixWeight));
        /*
        for (int i =0; i < meshes[0].vertices.size(); ++i){
                if(meshes[0].vertices[i].X.y > 0.8)
                fixConstraints.push_back(FixConstraint(i,qn,fixWeight));
            }
        */
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

      _LHS_LDLT.analyzePattern( leftSide );
      _LHS_LDLT.compute( leftSide );


  }

  int c = 0;
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
      for(auto& st : strainConstraints) {
        st.project(qn1);
      }
      #pragma omp parallel
      for(int i =0; i <10; ++i) {
        cout << i << endl;
      }
      #pragma omp parallel 
      for(auto& st : stretchConstraints) {
        st.project(qn1);
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

      //Global Solve
      qn1 = _LHS_LDLT.solve( rightSide );
    }


    for(auto& fc : fixConstraints) { fc.fix(qn1); }

    //Update velocity
    velocity = (qn1-qn) /h;
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





int main(int argc, char **argv) {

  cout<<"FUSING CONSTRAINT PROJECTION"<<endl;
  auto start = chrono::steady_clock::now();

  Solver solver;
  solver.initScene();

  //meshes[0].printVertexAndTrianglesAndEdges();
  auto initialisationTime = chrono::steady_clock::now();
  std::chrono::duration<double> initialisationSeconds = initialisationTime-start;

  for (int i = 0; i < nFrames; i++) {
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
