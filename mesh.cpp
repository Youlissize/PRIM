#ifndef MESH
#define MESH
#include "vector3.cpp"
#include <set>
#include <string>
using namespace std;


struct Vertex{
  Vec3f X;
  Vec3f P;
  float w;
  Vec3f vel;
  Vec3f acc;
  vector<tIndex> adjEdg; //adjacent edges
  vector<tIndex> adjTri; //adjacent triangles
};

struct Edge{
  tIndex A; //first vertex
  tIndex B; //other vertex
  vector<tIndex> adjTri; //adjacent triangles
};


struct Triangle{
  tIndex A; //first vertex
  tIndex B; //second vertex
  tIndex C; //third vertex
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
int compareEdges(Edge e1,Edge e2){  //compare two edges (assuming that edge.A<edge.B)
  if (e1.A<e2.A) { return -1;}
  if (e1.A==e2.A) {
    if (e1.B<e2.B) { return -1;}
    if (e1.B==e2.B) { return 0;}
    else { return 1;}
    }
  return 1;
}
int findEdge(vector<Edge> edges, Edge e){ //return the position in vector of e , dichotomic search
  int mid,left=0;
  int right = edges.size();
  while(left<right){
    mid = left + (right - left)/2;
    int res = compareEdges(e,edges[mid]);
    if (res > 0){
      left=mid+1;
    }
    else if (res<0) {
      right = mid;
    }
    else { return mid; }
  }
  cout<<"Edge not found"<<endl;
  return -1;
}


class Mesh{
public:
  string meshName;
  vector<Vertex> vertices;
  vector<Edge> edges;
  vector<Triangle> triangles;
  vector<vector<int>> trianglesTextures;
  vector<vector<int>> trianglesNormals;

 /* vector<Vec3f> X; //vertices position
  vector<Vec3f> P; //vertices temporary position during step
  vector<Vec3f> vel;  //velocity
  vector<Vec3f> acc; //acceleration
  vector<Real> w; //inverse of mass */
  bool isDeformable;

  string textures; //UV map of the texture
  string mtlFileString; //mtl File corresponding to the mesh
  string mtlName; //name of the material to put in .obj file

// importation and initialization
  Mesh (string FILENAME,bool isDeformable) {
    this->isDeformable = isDeformable;
    ifstream file(FILENAME);
    this-> meshName = FILENAME.substr(0,FILENAME.length() -4).substr(7);
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
                vertices.push_back({Vec3f(x, y, z),Vec3f(x, y, z),10,Vec3f(),Vec3f(),vector<tIndex>(),vector<tIndex>()});
/*                Vertex v;
                v.adjEdg = vector<tIndex>();
                v.adjTri = vector<tIndex>();
                vertices.push_back(v);*/
            }

            if (words[0].compare("vt") == 0 || words[0].compare("vn") == 0 || words[0].compare("usemtl") == 0){
                textures += line + '\n';
            }

            if (words[0].compare("f") == 0) {
                vector<string> vertex1, vertex2, vertex3;
                split(words[1],'/',back_inserter(vertex1));
                split(words[2],'/',back_inserter(vertex2));
                split(words[3],'/',back_inserter(vertex3));
                tIndex a = stol(vertex1[0])-1;
                tIndex b = stol(vertex2[0])-1;
                tIndex c = stol(vertex3[0])-1;
                Triangle tri;
                tri.A=a;
                tri.B=b;
                tri.C=c;
                tri.edges = vector<tIndex>();
                triangles.push_back(tri);
                vector<int> tex{stol(vertex1[1]),stol(vertex2[1]),stol(vertex3[1])};
                vector<int> norm{stol(vertex1[2]),stol(vertex2[2]),stol(vertex3[2])};
                trianglesTextures.push_back(tex);
                trianglesNormals.push_back(norm);

                // Fill edges
                if (a > b) { tempEdges.insert(make_pair(b, a));}
                else { tempEdges.insert(make_pair(a, b)); }

                if (c > b) { tempEdges.insert(make_pair(b, c)); }
                else { tempEdges.insert(make_pair(c, b)); }

                if (c > a) { tempEdges.insert(make_pair(a, c)); }
                else { tempEdges.insert(make_pair(c, a)); }
            }
        }

        // Fill all data of the mesh

/*        P = vector<Vec3f>(X.size());
        vel = vector<Vec3f>(X.size());
        acc = vector<Vec3f>(X.size());
        w = vector<Real>(X.size(),1); */ // Mass set by default at 1 for every vertex

        for (auto& e:tempEdges){
            Edge edge;
            edge.A=e.first;
            edge.B=e.second;
            edge.adjTri = vector<tIndex>();
            edges.push_back(edge);
        }

        //Fill edge.adjTri - edges is here supposed to be sorted
        for (tIndex i=0; i<triangles.size(); i++) {
          Triangle t = triangles[i];

          Edge e = {.A=min(t.A,t.B), .B=max(t.A,t.B)};
          int pos = findEdge(edges, e);
          edges[pos].adjTri.push_back(i);

          e = {.A=min(t.C,t.B), .B=max(t.C,t.B)};
          pos = findEdge(edges, e);
          edges[pos].adjTri.push_back(i);

          e = {.A=min(t.C,t.A), .B=max(t.C,t.A)};
          pos = findEdge(edges, e);
          edges[pos].adjTri.push_back(i);

        }

        //Fill triangle.edges
        for(tIndex i=0; i<edges.size(); i++){
          for(tIndex tri : edges[i].adjTri) {
            triangles[tri].edges.push_back(i);
          }
        }

        //Fill Vertices

        for(tIndex i=0; i<edges.size(); i++){
          vertices[edges[i].A].adjEdg.push_back(i);
          vertices[edges[i].B].adjEdg.push_back(i);
        }
        for(tIndex i=0; i<triangles.size(); i++){
          vertices[triangles[i].A].adjTri.push_back(i);
          vertices[triangles[i].B].adjTri.push_back(i);
          vertices[triangles[i].C].adjTri.push_back(i);
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
        ofstream myMtlFile;
        string name = "frame";
        string path = "Output/" + name;
        myMtlFile.open (path+ ".mtl");
        myMtlFile << mtlFileString;
        myMtlFile.close();
        cout << "Successfully imported " << mtlFILENAME << endl;}

    else {
        cout << "WARNING : Failed to import " << mtlFILENAME << endl;
    }



  }


  void printVertexAndTrianglesAndEdges() {

      cout << endl;
      cout << "---VERTICES---" << endl;
      for (tIndex i = 0; i < vertices.size(); i++) {
          cout << vertices[i].X << endl;
      }
      cout << "---TRIANGLES---" << endl;
      for (tIndex i = 0; i < triangles.size(); i++) {
          cout << triangles[i].A <<" / "<< triangles[i].B << " / " << triangles[i].C << endl;
      }
      cout << "---EDGES---" << endl;
      for (auto e: edges) {
          cout << e.A << "  " << e.B << endl;
      }
      cout << endl;

  }

  void exportToOBJ(int c){
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
    myfile << endl << "o " << meshName << endl;
    for (auto v:vertices){
      auto x= v.X;
      myfile <<"v "<<x.x<<" "<<x.y<<" "<<x.z<<endl;
    }

    myfile << textures;
    for (int t=0;t<triangles.size();t++){
        int x = trianglesTextures[t][0];
        int y = trianglesNormals[t][0];
        myfile<<"f "<<triangles[t].A +1<< "/" <<to_string(x) << "/"<< to_string(y) << " ";
        x = trianglesTextures[t][1];
        y = trianglesNormals[t][1];
        myfile<< triangles[t].B +1<< "/" <<to_string(x) << "/"<< to_string(y) << " ";
        x = trianglesTextures[t][2];
        y = trianglesNormals[t][2];
        myfile << triangles[t].C +1<< "/" <<to_string(x) << "/"<< to_string(y) << endl;
    }
    myfile.close();



  }



};



#endif
