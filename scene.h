#ifndef SCENE_H
#define SCENE_H
#include <stdlib.h>
#include<iostream>
#include "mesh.h"
#include <vector>
using namespace std;

class Scene
{
    public:
        Scene() {};

        void exportToOBJ(int c);
        void writeMTL();
        void setMeshes(vector<Mesh*> meshes){ this->meshes = meshes;}
      private:
          vector<Mesh*> meshes;
};

#endif // SCENE_H
