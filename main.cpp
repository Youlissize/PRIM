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

using namespace std;
typedef float Real;
typedef long int tIndex;



// Function declaration
void ImportMesh(string FILENAME);
void printVertexAndTrianglesAndEdges();




typedef pair<tIndex, tIndex> Edge;

// GLOBAL VARIABLES

vector<tIndex> triangles;
set<Edge> edges;        // edges are always (a,b) with a<b (to avoid repetition)
// vertex data
vector<Vec3f> _X;      // true position at the end of step
vector<Vec3f> _P;      // position approximation during step
vector<Vec3f> _vel;      // velocity
vector<Vec3f> _acc;      // acceleration
Real _defaultMass = 1;      // mass given for each vertex by default
vector<Real> _mass;      // mass of each vertex
vector<Real> _w;        // w[i] = 1/mass[i]

// simulation
int nFrames = 5;
Real _dt = 0.05;                     // time step

// Coefficients
Vec3f  _g = Vec3f(0, -9.8, 0);                    // gravity

string objectFile = "Meshes/cube.obj";       // Mesh to import

int solverIteration = 3;

// END GLOBAL VARIABLES





//Might be useful?
Vec3f VertexInterp( double isolevel,Vec3f p1,Vec3f p2,double valp1,double valp2)
{
   double mu;
   Vec3f p(0,0,0);

   if (fabs(isolevel-valp1) < 0.00001)
      return(p1);
   if (fabs(isolevel-valp2) < 0.00001)
      return(p2);
   if (fabs(valp1-valp2) < 0.00001)
      return(p1);
   mu = (isolevel - valp1) / (valp2 - valp1);
   p.x = p1.x + mu * (p2.x - p1.x);
   p.y = p1.y + mu * (p2.y - p1.y);
   p.z = p1.z + mu * (p2.z - p1.z);

   return(p);
}





class Solver {
public:
    //explicit Solver();

  void initScene() {

      //Initialize variables for importing
      _X = vector<Vec3f>();
      triangles = vector<tIndex>();
      edges = set<Edge>();

      // Import objects
      ImportMesh(objectFile);

      printVertexAndTrianglesAndEdges();


      //Compute and initialize other values
      _P = vector<Vec3f>(vertexCount(), Vec3f(0, 0, 0));
      _vel = vector<Vec3f>(vertexCount(), Vec3f(0, 0, 0));
      _acc = vector<Vec3f>(vertexCount(), Vec3f(0, 0, 0));
      _mass = vector<Real>(vertexCount(), _defaultMass);
      _w = vector<Real>(vertexCount(), 1);

      for (tIndex i = 0; i < _X.size(); i++) {
          _w[i] = 1 / _mass[i];
      }

  }
  int c = 0;

  void update() {
    cout << "." << flush;
    _acc = vector<Vec3f>(_X.size(), Vec3f(0, 0, 0));

    // apply External Force
    for (tIndex i = 0; i < vertexCount(); ++i) {
        _acc[i] += _g;
    }
    // update Velocity
#pragma omp parallel for
    for (tIndex i = 0; i < vertexCount(); ++i) {
        _vel[i] += _dt * _w[i] * _acc[i];   // simple forward Euler
    }

    dampVelocities();

    // update Temporary Position
#pragma omp parallel for
    for (tIndex i = 0; i < vertexCount(); ++i) {
        _P[i] = _X[i] + _dt * _vel[i];
    }

    generateCollisionConstraints();
    // Solve constraints
    for (int i = 0; i < solverIteration; i++) {
        projectConstraints();
    }

    for (tIndex i = 0; i < vertexCount(); ++i) {
        _vel[i] = (_P[i] - _X[i]) / _dt;
        _X[i] = _P[i];
    }
    velocityUpdate();
    writeOBJFile(c);
    c++;

  }

  tIndex vertexCount() const { return _X.size(); }
  const Vec3f& position(const tIndex i) const { return _X[i]; }


private:

  void dampVelocities() {
      // TODO
  }

  void generateCollisionConstraints() {
    // TODO
    }

  void projectConstraints() {
      // TODO
  }

  void velocityUpdate() {
      // TODO
  }


  //Compute isovalues to know if we are in or out an object
/* void updateGridIsovalues(){
  const Real sr = _kernel.supportRadius();

  #pragma omp parallel for
  for(tIndex i =0; i < gridcells.size(); i++) {
    for(tIndex k =0; k <8; k++){
      Vec3f xi = gridcells[i].p[k];
      Vec3f posMoy = Vec3f(0,0,0);
      double weightSum = 0;
      const int gi_from = static_cast<int>(xi.x-sr);
      const int gi_to   = static_cast<int>(xi.x+sr)+1;
      const int gj_from = static_cast<int>(xi.y-sr);
      const int gj_to   = static_cast<int>(xi.y+sr)+1;
      const int gk_from = static_cast<int>(xi.z-sr);
      const int gk_to   = static_cast<int>(xi.z+sr)+1;

      for(int gj=max(0, gj_from); gj<min(resY(), gj_to); ++gj) {
        for(int gi=max(0, gi_from); gi<min(resX(), gi_to); ++gi) {
          for(int gk=max(0, gk_from); gk<min(resZ(), gk_to); ++gk){

            const tIndex gidx = idx1d(gi, gj, gk);

            // each particle in nearby cells
            for(size_t ni=0; ni<_pidxInGrid[gidx].size(); ++ni) {
              const tIndex j = _pidxInGrid[gidx][ni];
              const Vec3f &xj = position(j);
              posMoy +=  _kernel.f((xj-xi).length())*xj;
              weightSum+= _kernel.f((xj-xi).length());
            }
          }
        }
      }
      if(weightSum!=0){
      posMoy = posMoy/weightSum;
      gridcells[i].val[k] = (xi-posMoy).length() - _h;
    }
    else gridcells[i].val[k] =1;

    }
    }
  } */

// Allows to write STL file from a list of triangles
  void writeOBJFile(int c){

    string path = "Output/frame";
    ofstream myfile;
    if( c < 10)
    myfile.open (path+"00"  + to_string(c) + ".obj");
    else {if( c >= 10 && c < 100)
    myfile.open (path+"0"  + to_string(c) + ".obj");
    else myfile.open (path + to_string(c) + ".obj");}

    myfile << "output frame" << endl;
    for (auto x:_X){
      myfile <<"v "<<x.x<<" "<<x.y<<" "<<x.z<<endl;
    }
    for (int t=0;t<triangles.size()/3;t++){
      myfile<<"f "<<triangles[3*t]<<" "<<triangles[3*t+1]<<" "<<triangles[3*t+2]<<endl;
    }
    myfile.close();

  }


  };

  // For importation
  template <typename Out>
  void split(const std::string& s, char delim, Out result) {
      std::istringstream iss(s);
      std::string item;
      while (std::getline(iss, item, delim)) {
          *result++ = item;
      }
  }

  // only working with .obj file
  void ImportMesh(string FILENAME) {

      ifstream file(FILENAME);
      if (file.is_open()) {
          string line;
          tIndex vertexOffset = _X.size(); //to avoid error when importing multiple meshes
          while (std::getline(file, line)) {

              vector<string> words;
              split(line, ' ', back_inserter(words));

              if (words[0].compare("v") == 0) {
                  float x = stof(words[1]);
                  float y = stof(words[2]);
                  float z = stof(words[3]);
                  _X.push_back(Vec3f(x, y, z));
              }

              if (words[0].compare("f") == 0) {
                  tIndex a = stol(words[1]) + vertexOffset;
                  tIndex b = stol(words[2]) + vertexOffset;
                  tIndex c = stol(words[3]) + vertexOffset;
                  triangles.push_back(a);
                  triangles.push_back(b);
                  triangles.push_back(c);

                  // Fill edges
                  if (a > b) { edges.insert(make_pair(b, a));}
                  else { edges.insert(make_pair(a, b)); }

                  if (c > b) { edges.insert(make_pair(b, c)); }
                  else { edges.insert(make_pair(c, b)); }

                  if (c > a) { edges.insert(make_pair(a, c)); }
                  else { edges.insert(make_pair(c, a)); }

              }
          }
          file.close();
          cout << "Successfully imported " << FILENAME << endl;
      }
      else {
          cout << "WARNING : Failed to import " << FILENAME << endl;
      }

      return;


  }


  // Utilities

  void printVertexAndTrianglesAndEdges() {
      cout << endl;
      cout << "---VERTICES---" << endl;
      for (tIndex i = 0; i < _X.size(); i++) {
          cout << _X[i] << endl;
      }
      cout << "---TRIANGLES---" << endl;
      for (tIndex i = 0; i < triangles.size()/3; i++) {
          cout << triangles[3*i] <<" / "<< triangles[3 * i+1] << " / " << triangles[3 * i+2] << endl;
      }
      cout << "---EDGES---" << endl;
      for (auto e: edges) {
          cout << e.first << "  " << e.second << endl;
      }
      cout << endl;

  }










int main(int argc, char **argv) {

  Solver solver;
  solver.initScene();

  for (int i = 0; i < nFrames; i++) {
      solver.update();
  }

  cout << "State after " << nFrames << " frames : " << endl;
  printVertexAndTrianglesAndEdges();

  return EXIT_SUCCESS;
}
