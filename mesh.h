#ifndef MESH
#define MESH

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include<set>
#include "vector3.h"

using namespace std;

struct Vertex{
  Vec3f X;
  Vec3f P;
  float w;
  Vec3f vel;
  Vec3f acc;
  vector<int> adjEdg; //adjacent edges
  vector<int> adjTri; //adjacent triangles
};

struct Edge{
  int A; //first vertex
  int B; //other vertex
  vector<int> adjTri; //adjacent triangles
};


struct Triangle{
  int A; //first vertex
  int B; //second vertex
  int C; //third vertex
  vector<int> edges; //edges of the triangles
};


struct Tetra {
  int A;
  int B;
  int C;
  int D;
};



class Mesh{
public:
  long meshVertices= 0;
  long meshTextures= 0;
  long meshNormals = 0;
  string meshName;
  vector<Vertex> vertices;
  vector<Edge> edges;
  vector<Triangle> triangles;
  vector<Tetra> tetras;
  vector<vector<int>> trianglesTextures;
  vector<vector<int>> trianglesNormals;

 /* vector<Vec3f> X; //vertices position
  vector<Vec3f> P; //vertices temporary position during step
  vector<Vec3f> vel;  //velocity
  vector<Vec3f> acc; //acceleration
  vector<Real> w; //inverse of mass */
  bool isDeformable;
  bool isTetraedral;

  string textures; //UV map of the texture
  string mtlFileString; //mtl File corresponding to the mesh
  string mtlName; //name of the material to put in .obj file

  Mesh (string FILENAME,bool isDeformable,float vertexWeight);
  void printVertexAndTrianglesAndEdges();
  void writeNodeFile();
  string exportToOBJ(long totalVertices, long totalTextures, long totalNormals, int c);


};


#endif
