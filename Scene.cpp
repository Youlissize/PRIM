#ifndef SCENE_H
#define SCENE_H
#include "mesh.cpp"
#include <vector>

class Scene
{
    public:
        Scene(vector<Mesh> meshes){
        this->meshes = meshes;
        }

        void exportToObj(){

        }
    private:
        vector<Mesh> meshes;
};

#endif // SCENE_H

