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
#include "fusingConstraint.cpp"
#include "Scene.cpp"
#include "linearSystem.h"
#include "eigen-3.3.8/Eigen/Sparse"
#include "eigen-3.3.8/Eigen/Dense"
#include "eigen-3.3.8/Eigen/SVD"
#include "myVec3.cpp"

using namespace std;
typedef float Real;
typedef long int tIndex;
typedef Eigen::Matrix<float,Eigen::Dynamic,1> floatVector;
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

vector<FusingConstraint> constraints = vector<FusingConstraint>();  // Ci in paper
vector<Vec3Vector> projections = vector<Vec3Vector>();    // {pi} in paper

// END GLOBAL VARIABLES



// UTILITIES (merci JMT <3)

class MySparseMatrix {
    std::vector< std::map<unsigned int , float> > _ASparse;

    unsigned int _rows , _columns;

public:
    MySparseMatrix() {
        _rows = _columns = 0;
    }
    MySparseMatrix( int rows , int columns ) {
        setDimensions(rows , columns);
    }
    ~MySparseMatrix() {
    }

    std::map< unsigned int , float > const & getRow( unsigned int r ) const { return _ASparse[r]; }

    void setDimensions( int rows , int columns ) {
        _rows = rows; _columns = columns;
        _ASparse.clear();
        _ASparse.resize(_rows);
    }

    float & operator() (unsigned int row , unsigned int column) {
        return _ASparse[row][column];
    }

    void convertToEigenFormat(Eigen::SparseMatrix<float> & _A) {
        // convert ad-hoc matrix to Eigen sparse format:
        {
            _A.resize(_rows , _columns);
            std::vector< Eigen::Triplet< float > > triplets;
            for( unsigned int r = 0 ; r < _rows ; ++r ) {
                for( std::map< unsigned int , float >::const_iterator it = _ASparse[r].begin() ; it != _ASparse[r].end() ; ++it ) {
                    unsigned int c = it->first;
                    float val = it->second;
                    triplets.push_back( Eigen::Triplet< float >(r,c,val) );
                }
            }
            _A.setFromTriplets( triplets.begin() , triplets.end() );
        }
    }
};

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
      meshes.push_back(Mesh(objectFile,true));
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
      //TODO


      //Precompute system for global solving
      SparseMat leftSide = M / (h*h); //Matrix on the left side of equation (10)

      for (auto& c : constraints) {
        if(c.AandBareIdentity){
          leftSide += c.w * c.S.transpose() * c.S;
        }
        else {
          leftSide += c.w * c.S.transpose() * c.A.transpose() * c.A * c.S;
        }
      }
      Eigen::SimplicialLDLT< Eigen::SparseMatrix<float> > _LHS_LDLT;
      _LHS_LDLT.analyzePattern( leftSide );
      _LHS_LDLT.compute( leftSide );


  }

  int c = 0;

  void update() {
    cout << c << " " << flush;
    // Compute Sn

    sn = qn + velocity *h + M_inv*fext * h*h;
    qn1 = sn;

    //Main solver loop

    for (int loopCount=0;loopCount<solverIteration;loopCount++){

      //Local constraints solve
      for(int i =0; i<constraints.size();i++) {
          //projections[i] = constraints[i].project(nextPos);
      }

      //Global Solve
      globalSolve();
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

  void globalSolve() {
    //TODO
  }

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
