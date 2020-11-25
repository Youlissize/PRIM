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
#include "mesh.cpp"
#include "vector3.cpp"
#include "mySparseMatrix.cpp"
#include "fusingConstraint.cpp"
#include "Scene.cpp"
#include "myVec3.cpp"

using namespace std;
typedef float Real;
typedef long int tIndex;
typedef Eigen::Matrix<float,Eigen::Dynamic,1> FloatVector;
typedef Eigen::Matrix<MyVec3,Eigen::Dynamic,1> Vec3Vector;
typedef Eigen::SparseMatrix<float> SparseMat;
typedef Eigen::SparseMatrix<MyVec3> Vec3SparseMat;
typedef Eigen::DiagonalMatrix<float,Eigen::Dynamic> DiagMatrix;



// GLOBAL VARIABLES

// Objects
vector<Mesh> meshes;
Scene* scene = new Scene();
string objectFile = "Meshes/blueCube.obj";       // Mesh to import
string floorFile = "Meshes/floor.obj";

// simulation
int nFrames = 100;
Real h = 0.05;                     // time step
int solverIteration = 5;

// Coefficients
Vec3f  _g = Vec3f(0, -9.8, 0);                    // gravity

// Variables
tIndex N; //total number of vertices
Vec3Vector qn;  //q_n in paper
Vec3Vector qn1;  //q_n+1 in paper
Vec3Vector sn;       //s_n in paper
Vec3Vector velocity; //v_n in paper
SparseMat M; //diagonal matrix of mass
SparseMat M_inv; //diagonal matrix of 1/mass
Vec3Vector fext; //external forces
Eigen::SimplicialLDLT< Eigen::SparseMatrix<float> > _LHS_LDLT; //LinearSystem solver

//Constraints
tIndex N_constraints;
vector<StretchConstraint> stretchConstraints = vector<StretchConstraint>();  // Ci in paper


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
      meshes.push_back(Mesh(objectFile,true,1.0f));
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
      qn = Vec3Vector(N);
      velocity = Vec3Vector(N);
      fext = Vec3Vector(N);
      MySparseMatrix M_mine( N, N );
      MySparseMatrix M_inv_mine( N, N );

      tIndex count = 0;
      for(auto& mesh:meshes){
        for(auto& v:mesh.vertices) {
          qn[count]= MyVec3(v.X);
          velocity[count]  = MyVec3(0,0,0);
          fext[count] = _g;
          M_mine(count,count)=1.0f/v.w;
          M_inv_mine(count,count)=v.w;
          count ++;
        }
      }
      M_mine.convertToEigenFormat(M);
      M_inv_mine.convertToEigenFormat(M_inv);
      qn1 = Vec3Vector(N);
      sn = Vec3Vector(N);



      // Create constraints
      N_constraints = 0;
      //StretchConstraints
      float stretchWeight = 0.5f;
      tIndex offset = 0;

      for(auto& mesh : meshes){
        for(auto& e :mesh.edges){
          stretchConstraints.push_back( StretchConstraint(e.A+offset,e.B+offset,qn,stretchWeight) );
          break;
        }
        offset += mesh.meshVertices;
      }

      N_constraints += stretchConstraints.size();


      //Precompute system for global solving
      SparseMat leftSide = M / (h*h); //Matrix on the left side of equation (10)

      for (auto& c : stretchConstraints) {
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
    // !!!! SparseMat*Vec3Vector return wrong answer
    qn1 = sn;


    //Main solver loop
    SparseMat tmp = M / (h*h);

    for (int loopCount=0;loopCount<solverIteration;loopCount++){
      Vec3Vector rightSide = tmp.diagonal().asDiagonal()*sn; // right side of equation (10)
      //Local constraints solve (could be done in parralel)
      for(int i =0; i<stretchConstraints.size();i++) {
        stretchConstraints[i].project(qn1);
      }

      // update rightSide
      for(int i =0; i<stretchConstraints.size();i++) {
          stretchConstraints[i].addProjection(rightSide);
      }


      //Global Solve
      qn1 = _LHS_LDLT.solve( rightSide );
    }

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
    tIndex count = 0;
    for(auto& mesh: meshes) {
      for(auto& v:mesh.vertices){
        v.X = Vec3f(qn[count].x,qn[count].y,qn[count].z);
        count ++;
      }
    }
  }

  void writeOBJFile(int c){
    scene->exportToOBJ(c);
  }

};





int main(int argc, char **argv) {
  cout<<"FUSING CONSTRAINT PROJECTION"<<endl<<endl;

  Solver solver;
  solver.initScene();

  //meshes[0].printVertexAndTrianglesAndEdges();

  for (int i = 0; i < nFrames; i++) {
      solver.update();
      scene->exportToOBJ(i);
  }
scene->writeMTL();
  cout << "State after " << nFrames << " frames : " << endl;
  //meshes[0].printVertexAndTrianglesAndEdges();





  return EXIT_SUCCESS;
}
