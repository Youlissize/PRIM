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
int nFrames = 250;
Real _dt = 0.05;                     // time step
int solverIteration = 15;
float streching = 0.9; //streching
float k_streching = 1.f-pow((1.f-streching),1.f/solverIteration);
float bending = 0.8; //bending
float k_bending = 1.f-pow((1.f-bending),1.f/solverIteration);
float bouncing = 0.8; //bouncing
float k_bouncing = 1.f-pow((1.f-bouncing),1.f/solverIteration);


// Coefficients
Vec3f  _g = Vec3f(0, -9.8, 0);                    // gravity

// Constraints
vector<LengthConstraint> lengthConstraints =  vector<LengthConstraint>();
vector<FixConstraint> fixConstraints = vector<FixConstraint>();
vector<BendConstraint> bendConstraints = vector<BendConstraint>();
vector<CollisionConstraint> collisionConstraint = vector<CollisionConstraint>();

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

      //Import meshes
      meshes = vector<Mesh>();
      meshes.push_back(Mesh(objectFile,true));
      meshes.push_back(Mesh(floorFile,false));
      vector<Mesh*> meshesPointers = vector<Mesh*>();
      for (int i =0; i< meshes.size();++i){
        meshesPointers.push_back(&meshes[i]);
      }
      scene->setMeshes(meshesPointers);
     /* for (int i =0; i<1; ++i)
      fixConstraints.push_back(FixConstraint(&meshes[0].vertices[4]));
      fixConstraints.push_back(FixConstraint(&meshes[0].vertices[0]));*/
      for (int i =0; i < meshes.size();++i){
          if(meshes[i].isDeformable){
              for (auto e : meshes[i].edges){
                lengthConstraints.push_back(LengthConstraint(&meshes[i].vertices[e.A],&meshes[i].vertices[e.B],k_streching));
              }

              for (auto e : meshes[i].edges){
                if (e.adjTri.size() == 2) {
                  Vertex * v3;
                  Vertex * v4; // The two points that are not on the edge
                  Triangle t1 = meshes[i].triangles[e.adjTri[0]];
                  if (t1.A!=e.A && t1.A!=e.B){ v3 = &meshes[i].vertices[t1.A]; }
                  else if (t1.B!=e.A && t1.B!=e.B){ v3 = &meshes[i].vertices[t1.B]; }
                  else if (t1.C!=e.A && t1.C!=e.B){ v3 = &meshes[i].vertices[t1.C]; }

                  Triangle t2 = meshes[i].triangles[e.adjTri[1]];
                  if (t2.A!=e.A && t2.A!=e.B){ v4 = &meshes[i].vertices[t2.A]; }
                  else if (t2.B!=e.A && t2.B!=e.B){ v4 = &meshes[i].vertices[t2.B]; }
                  else if (t2.C!=e.A && t2.C!=e.B){ v4 = &meshes[i].vertices[t2.C]; }

                  bendConstraints.push_back(BendConstraint(&meshes[i].vertices[e.A],&meshes[i].vertices[e.B],v3,v4,k_bending));
                }
              }
            }
        }
  }

  int c = 0;

  void update() {
    cout << c << " " << flush;

    // apply gravity
    for(auto& mesh:meshes) {
      for (tIndex i = 0; i < mesh.vertices.size(); ++i) {
        mesh.vertices[i].acc = _g;
      }
    }


    // update Velocity
    for(auto& mesh:meshes) {
//  #pragma omp parallel for
    if(mesh.isDeformable){
          for (tIndex i = 0; i < mesh.vertices.size(); ++i) {
              mesh.vertices[i].vel += _dt * (1.f/mesh.vertices[i].w) * mesh.vertices[i].acc;   // simple forward Euler
          }
        }
      }
    dampVelocities();

    // update Temporary Position
    for(auto& mesh:meshes) {
  #pragma omp parallel for
      if(mesh.isDeformable){
          for (tIndex i = 0; i < mesh.vertices.size(); ++i) {
              mesh.vertices[i].P = mesh.vertices[i].X + _dt * mesh.vertices[i].vel;
          }
        }
    }


    generateCollisionConstraints();

    // Solve constraints
    for (int i = 0; i < solverIteration; i++) {
        projectConstraints();
    }
for (int i = 0; i < solverIteration; i++) {
    projectCollisionConstraints();}

    for(auto& mesh:meshes) {
        if(mesh.isDeformable){
          for (tIndex i = 0; i < mesh.vertices.size(); ++i) {
              mesh.vertices[i].vel = (mesh.vertices[i].P - mesh.vertices[i].X) / _dt;
              mesh.vertices[i].X = mesh.vertices[i].P;
          }
        }
    }

    velocityUpdate();
    writeOBJFile(c);
    c++;



  }


private:

  void dampVelocities() {
      // TODO
  }

  pair<bool,Vec3f> intersects(Vertex V, Mesh mesh, Triangle T){
      //Algorithme de Möller Trumbore
    const float EPSILON = 0.0000001;
    Vec3f vertex0 = mesh.vertices[T.A].X;
    Vec3f vertex1 = mesh.vertices[T.B].X;
    Vec3f vertex2 = mesh.vertices[T.C].X;
    Vec3f rayVector = V.P-V.X;
    Vec3f rayOrigin = V.X;
    Vec3f edge1, edge2, h, s, q;
    float a,f,u,v;
    edge1 = vertex1 - vertex0;
    edge2 = vertex2 - vertex0;
    h = rayVector.crossProduct(edge2);
    a = edge1.dotProduct(h);
    if (abs(a) < EPSILON)
        return {false,Vec3f()};    // Le rayon est parallèle au triangle.

    f = 1.0/a;
    s = rayOrigin - vertex0;
    u = f * (s.dotProduct(h));
    if (u < 0.0 || u > 1.0)
        return {false,Vec3f()};
    q = s.crossProduct(edge1);
    v = f * rayVector.dotProduct(q);
    if (v < 0.0 || u + v > 1.0)
        return {false,Vec3f()};

    // On calcule t pour savoir ou le point d'intersection se situe sur la ligne.
    float t = f * edge2.dotProduct(q);
    if (t > EPSILON) // Intersection avec le rayon
    {
        Vec3f intersectionPoint = rayOrigin + t*rayVector ;
        if(rayVector.length()*2- (intersectionPoint - V.X).length()>0){
            return {true,intersectionPoint};

        }
        else
            return {false,Vec3f()};
    }
    else // On a bien une intersection de droite, mais pas de rayon.
        return {false,Vec3f()};
}


  void generateCollisionConstraints() {
      collisionConstraint.clear();
    for (int j = 0; j < meshes.size(); j++){
        if (meshes[j].isDeformable){
            for (Mesh rigid : meshes){
                if (!rigid.isDeformable){
                    for (int i =0; i < meshes[j].vertices.size(); i++){
                        for (Triangle T : rigid.triangles){
                            pair<bool,Vec3f> intersect =intersects(meshes[j].vertices[i],rigid,T) ;
                            if (intersect.first){
                                collisionConstraint.push_back(CollisionConstraint(&meshes[j].vertices[i],intersect.second,rigid.vertices[T.A],rigid.vertices[T.B],rigid.vertices[T.C],k_bouncing));
                            }
                        }
                    }
                }
            }
        }
    }
    }

  void projectConstraints() {
    for (auto fc: fixConstraints){
      fc.project();
    }

    for (auto lc: lengthConstraints){
      lc.project();
    }
    for (auto bc: bendConstraints){
      bc.project();
    }
  }

    void projectCollisionConstraints() {
        for (auto cc: collisionConstraint){
            cc.project();
        }
    }



  void velocityUpdate() {
      // TODO
  }

  void writeOBJFile(int c){
    scene->exportToOBJ(c);
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
