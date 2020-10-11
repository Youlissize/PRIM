#ifndef MESH
#define MESH
#include "vector3.cpp"
#include <set>
using namespace std;


struct Vertex{
  vector<tIndex> adjEdg; //adjacent edges
  vector<tIndex> adjTri; //adjacent triangles
};

struct Edge{
  tIndex A; //first vertex
  tIndex B; //other vertex
  vector<tIndex> adjTri; //adjacent triangles
  vector<tIndex> adjEdg; //adjacent edges
};


struct Triangle{
  tIndex A; //first vertex
  tIndex B; //second vertex
  tIndex C; //third vertex
  vector<tIndex> adjTri; //adjacent triangles
  vector<tIndex> edges; //edges of the triangles
};





// for importation
template <typename Out>
void split(const std::string& s, char delim, Out result) {
    std::istringstream iss(s);
    std::string item;
    while (std::getline(iss, item, delim)) {
        *result++ = item;
    }
}


class Mesh{
public:
  vector<Vertex> vertices;
  vector<Edge> edges;
  vector<Triangle> triangles;

  vector<Vec3f> X; //vertices position
  vector<Vec3f> P; //vertices temporary position during step
  vector<Vec3f> vel;  //velocity
  vector<Vec3f> acc; //acceleration
  vector<Real> w; //inverse of mass
  string textures; //UV map of the texture
  string mtlFileString; //mtl File corresponding to the mesh
  string mtlName; //name of the material to put in .obj file
  int test;

// importation and initialization
  Mesh (string FILENAME) {
    ifstream file(FILENAME);

    if (file.is_open()) {
        string line;
        set<pair<tIndex,tIndex>> tempEdges = set<pair<tIndex,tIndex>>();
        while (std::getline(file, line)) {

            vector<string> words;
            split(line, ' ', back_inserter(words));

            if (words[0].compare("v") == 0) {
                float x = stof(words[1]);
                float y = stof(words[2]);
                float z = stof(words[3]);
                X.push_back(Vec3f(x, y, z));
            }

            if (words[0].compare("vt") == 0){
                textures += line + '\n';
            }

            if (words[0].compare("f") == 0) {
                tIndex a = stol(words[1]);
                tIndex b = stol(words[2]);
                tIndex c = stol(words[3]);
                Triangle tri;
                tri.A=a;
                tri.B=b;
                tri.C=c;
                triangles.push_back(tri);

                // Fill edges
                if (a > b) { tempEdges.insert(make_pair(b, a));}
                else { tempEdges.insert(make_pair(a, b)); }

                if (c > b) { tempEdges.insert(make_pair(b, c)); }
                else { tempEdges.insert(make_pair(c, b)); }

                if (c > a) { tempEdges.insert(make_pair(a, c)); }
                else { tempEdges.insert(make_pair(c, a)); }
            }
        }
        file.close();
        cout << "Successfully imported " << FILENAME << endl;
    }
    else {
        cout << "WARNING : Failed to import " << FILENAME << endl;
    }
    string mtlFILENAME = FILENAME.substr(0,FILENAME.length() -4) + ".mtl";
    ifstream mtlFile(mtlFILENAME);
        if (mtlFile.is_open()) {
        string line;
        while (std::getline(mtlFile, line)) {
                        mtlFileString += line + "\n";
                        if (line.substr(0,6).compare("newmtl") == 0)
                            mtlName = line.substr(7);
            }

        file.close();
        cout << "Successfully imported " << mtlFILENAME << endl;}

    else {
        cout << "WARNING : Failed to import " << mtlFILENAME << endl;
    }

    P = vector<Vec3f>(X.size());
    vel = vector<Vec3f>(X.size());
    acc = vector<Vec3f>(X.size());
    w = vector<Real>(X.size(),1); // Mass set by default at 1 for every vertex
    this->test = 1000;
    // TODO : edges

  }


  void printVertexAndTrianglesAndEdges() {

      cout << endl;
      cout << "---VERTICES---" << endl;
      for (tIndex i = 0; i < X.size(); i++) {
          cout << X[i] << endl;
      }
      cout << "---TRIANGLES---" << endl;
      for (tIndex i = 0; i < triangles.size(); i++) {
          cout << triangles[i].A <<" / "<< triangles[i].B << " / " << triangles[i].C << endl;
      }
      /*
      cout << "---EDGES---" << endl;
      for (auto e: edges) {
          cout << e.A << "  " << e.B << endl;
      }
      cout << endl;*/

  }

  void exportToOBJ(int c){
    string name = "frame";
    string path = "Output/" + name;
    ofstream myfile;
    if( c < 10){
        myfile.open (path+"00"  + to_string(c) + ".obj");
        myfile << "mtllib " + name + "00" + to_string(c) + ".stl";
    }
    else {if( c >= 10 && c < 100){
            myfile.open (path+"0"  + to_string(c) + ".obj");
            myfile << "mtllib " + name + "0" + to_string(c) + ".stl";
        }
        else{
                myfile.open (path + to_string(c) + ".obj");
                myfile << "mtllib " + name + to_string(c) + ".stl";}
    }
    myfile << endl << "output frame" << endl;
    for (auto x:X){
      myfile <<"v "<<x.x<<" "<<x.y<<" "<<x.z<<endl;
    }

    myfile << textures;
    myfile << "usemtl " +mtlName << endl;
    for (int t=0;t<triangles.size()/3;t++){
      myfile<<"f "<<triangles[t].A<<" "<<triangles[t].B<<" "<<triangles[t].C<<endl;
    }
    myfile.close();


    ofstream myMtlFile;
    if( c < 10)
    myMtlFile.open (path+"00"  + to_string(c) + ".mtl");
    else {if( c >= 10 && c < 100)
    myMtlFile.open (path+"0"  + to_string(c) + ".mtl");
    else myMtlFile.open (path + to_string(c) + ".mtl");}
    myMtlFile << mtlFileString;
    myMtlFile.close();
  }



};



#endif
