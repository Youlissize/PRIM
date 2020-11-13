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
#include "constraint.cpp"
#include "Scene.cpp"
#include "linearSystem.h"

using namespace std;
typedef float Real;
typedef long int tIndex;




// GLOBAL VARIABLES

// Objects
vector<Mesh> meshes;
Scene* scene = new Scene();
string objectFile = "Meshes/texturedSphere.obj";       // Mesh to import
string floorFile = "Meshes/floor.obj";

// simulation
int nFrames = 0;
Real _dt = 0.05;                     // time step
int solverIteration = 5;

// Coefficients
Vec3f  _g = Vec3f(0, -9.8, 0);                    // gravity

// END GLOBAL VARIABLES



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


  }

  int c = 0;

  void update() {
    cout << c << " " << flush;


    writeOBJFile(c);
    c++;



  }


private:


  void writeOBJFile(int c){
    scene->exportToOBJ(c);
  }

};





int main(int argc, char **argv) {

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
