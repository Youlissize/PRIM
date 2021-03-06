#include "scene.h"
using namespace std;

        void Scene::exportToOBJ(int c){
            long totalVertices= 0;
            long totalTextures= 0;
            long totalNormals=0 ;
            string name = "frame";
            string path = "Output/" + name;
            ofstream myfile;
            if( c < 10){
                myfile.open (path+"00"  + to_string(c) + ".obj");
                myfile << "mtllib " + name  + ".mtl";
            }
            else {if( c >= 10 && c < 100){
                    myfile.open (path+"0"  + to_string(c) + ".obj");
                    myfile << "mtllib " + name  + ".mtl";
                }
                else{
                        myfile.open (path + to_string(c) + ".obj");
                        myfile << "mtllib " + name + ".mtl";}
            }
            myfile << endl;
            for (Mesh* mesh : meshes){
                myfile << mesh->exportToOBJ(totalVertices,totalTextures,totalNormals,c);
                totalVertices += mesh->meshVertices;
                totalTextures+= mesh->meshTextures;
                totalNormals+= mesh->meshNormals;
            }
        myfile.close();
        }

        void Scene::writeMTL(){

        ofstream myMtlFile;
        string name = "frame";
        string path = "Output/" + name;
        myMtlFile.open (path+ ".mtl");
        for(Mesh* mesh : meshes)
            myMtlFile << mesh->mtlFileString;
        myMtlFile.close();}

        
