#include "mesh.h"
using namespace std;


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




// importation and initialization
  Mesh::Mesh (string FILENAME,bool isDeformable,float vertexWeight) {
    this->isDeformable = isDeformable;
    ifstream file(FILENAME);

    // .obj Format
    if(FILENAME.find(".obj") != string::npos) {
      this-> isTetraedral = false;
      this-> meshName = FILENAME.substr(0,FILENAME.length() -4).substr(7) + "frame";
      if (file.is_open()) {
          string line;
          set<pair<int,int>> tempEdges = set<pair<int,int>>();
          float maxY = 0.f , maxX = 0.f, minX = 100.f , minY = 100.f;
          int maxYIndex = -1;
          while (std::getline(file, line)) {
                  //cout << line << endl;
              vector<string> words;
              split(line, ' ', back_inserter(words));
          for (int i =0; i < words.size(); i++){
              if (words[i] == "") words.erase(words.begin() + i);
          }

              if (words[0].compare("v") == 0) {
                  float x = stof(words[1]);
                  float y = stof(words[2]);
                  float z = stof(words[3]);
                  if (z > maxY) maxY = z;
                  if (z < minY) minY = z;
                  vertices.push_back({Vec3f(x, y, z),Vec3f(x, y, z),1.f/vertexWeight,Vec3f(),Vec3f(),vector<int>(),vector<int>()});
                  meshVertices++;
              }

              if(words[0].compare("usemtl") == 0){
                  textures += line + '\n';
              }
               if (words[0].compare("vn") == 0){
                  textures += line + '\n';
                  meshNormals++;
              }

              if (words[0].compare("vt") == 0){
                  textures += line + '\n';
                  meshTextures++;
              }

              if (words[0].compare("f") == 0) {
                  vector<string> vertex1, vertex2, vertex3;
                  split(words[1],'/',back_inserter(vertex1));
                  split(words[2],'/',back_inserter(vertex2));
                  split(words[3],'/',back_inserter(vertex3));
                  int a = stol(vertex1[0])-1;
                  int b = stol(vertex2[0])-1;
                  int c = stol(vertex3[0])-1;
                  Triangle tri;
                  tri.A=a;
                  tri.B=b;
                  tri.C=c;
                  tri.edges = vector<int>();
                  triangles.push_back(tri);
                  if(vertex1[1] != ""){
                  vector<int> tex{(int)stol(vertex1[1]),(int)stol(vertex2[1]),(int)stol(vertex3[1])};
                  trianglesTextures.push_back(tex);}
                  if(vertex1[2] != ""){
                  vector<int> norm{(int)stol(vertex1[2]),(int)stol(vertex2[2]),(int)stol(vertex3[2])};
                  trianglesNormals.push_back(norm);}


                  // Fill edges
                  if (a > b) { tempEdges.insert(make_pair(b, a));}
                  else { tempEdges.insert(make_pair(a, b)); }

                  if (c > b) { tempEdges.insert(make_pair(b, c)); }
                  else { tempEdges.insert(make_pair(c, b)); }

                  if (c > a) { tempEdges.insert(make_pair(a, c)); }
                  else { tempEdges.insert(make_pair(c, a)); }
              }
          }
          cout << maxY<< " * * " << minY<<  endl;


          // Fill all data of the mesh
          for (auto& e:tempEdges){
              Edge edge;
              edge.A=e.first;
              edge.B=e.second;
              edge.adjTri = vector<int>();
              edges.push_back(edge);
          }

          //Fill Vertices Data
          for(int i=0; i<edges.size(); i++){
            vertices[edges[i].A].adjEdg.push_back(i);
            vertices[edges[i].B].adjEdg.push_back(i);
          }
          for(int i=0; i<triangles.size(); i++){
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
              }

          file.close();
          cout << "Successfully imported " << mtlFILENAME << endl;}

      else {
          cout << "WARNING : Failed to import " << mtlFILENAME << endl;
      }

    }


    // .mesh Format
    else if(FILENAME.find(".mesh") != string::npos) {
      cout<<"mesh format"<<endl;
      this-> isTetraedral = true;
      this-> meshName = FILENAME.substr(0,FILENAME.length() -5).substr(7) + "frame";
      if (file.is_open()) {
          string line;
          vector<string> words;
          set<pair<int,int>> tempEdges = set<pair<int,int>>();

          while (std::getline(file, line)) {
              words.clear();
              split(line, ' ', back_inserter(words));
              for (int i =0; i < words.size(); i++){
                if (words[i] == "") words.erase(words.begin() + i);
              }

              if (words[0].compare("Vertices") == 0) {
                words.clear();
                std::getline(file,line);
                this->meshVertices = stoi(line);

                for (int i = 0; i<this->meshVertices; i++){
                  words.clear();
                  std::getline(file, line);
                  split(line, ' ', back_inserter(words));
                  for (int i =0; i < words.size(); i++){
                    if (words[i] == "") words.erase(words.begin() + i);
                  }
                  float x = stof(words[0]);
                  float y = stof(words[1]);
                  float z = stof(words[2]);
                  vertices.push_back({Vec3f(x, y, z),Vec3f(x, y, z),1.f/vertexWeight,Vec3f(),Vec3f(),vector<int>(),vector<int>()});
                }
              }

              else if (words[0].compare("Triangles") == 0) {
                words.clear();
                std::getline(file,line);
                int nbTriangles = stoi(line);

                for (int i = 0; i<nbTriangles; i++){
                  words.clear();
                  std::getline(file, line);
                  split(line, ' ', back_inserter(words));
                  for (int i =0; i < words.size(); i++){
                    if (words[i] == "") words.erase(words.begin() + i);
                  }
                  int a = stoi(words[0])-1;
                  int b = stoi(words[1])-1;
                  int c = stoi(words[2])-1;
                  triangles.push_back({a,b,c,vector<int>()});

                  // Fill edges
                  if (a > b) { tempEdges.insert(make_pair(b, a));}
                  else { tempEdges.insert(make_pair(a, b)); }

                  if (c > b) { tempEdges.insert(make_pair(b, c)); }
                  else { tempEdges.insert(make_pair(c, b)); }

                  if (c > a) { tempEdges.insert(make_pair(a, c)); }
                  else { tempEdges.insert(make_pair(c, a)); }
                }
              }

              else if (words[0].compare("Tetrahedra") == 0) {
                words.clear();
                getline(file,line);
                int nbTetra = stoi(line);

                for (int i = 0; i<nbTetra; i++){
                  words.clear();
                  std::getline(file, line);
                  split(line, ' ', back_inserter(words));
                  for (int i =0; i < words.size(); i++){
                    if (words[i] == "") words.erase(words.begin() + i);
                  }
                  int a = stoi(words[0])-1;
                  int b = stoi(words[1])-1;
                  int c = stoi(words[2])-1;
                  int d = stoi(words[3])-1;
                  tetras.push_back({a,b,c,d});

                }
              }

          }

          // Fill Edges
          for (auto& e:tempEdges){
              Edge edge;
              edge.A=e.first;
              edge.B=e.second;
              edge.adjTri = vector<int>();
              edges.push_back(edge);
          }

          //Fill Vertices Data
          for(int i=0; i<edges.size(); i++){
            vertices[edges[i].A].adjEdg.push_back(i);
            vertices[edges[i].B].adjEdg.push_back(i);
          }
          for(int i=0; i<triangles.size(); i++){
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
    }

    // Wrong Format
    else {
      cout<<"WARNING : File Format not supported"<<endl;
    }

  }


  void Mesh::printVertexAndTrianglesAndEdges() {

      cout << endl;
      cout << "---VERTICES---" << endl;
      for (int i = 0; i < vertices.size(); i++) {
          cout << vertices[i].X << endl;
      }
      cout << "---TRIANGLES---" << endl;
      for (int i = 0; i < triangles.size(); i++) {
          cout << triangles[i].A <<" / "<< triangles[i].B << " / " << triangles[i].C << endl;
      }
      cout << "---EDGES---" << endl;
      for (auto e: edges) {
          cout << e.A << "  " << e.B << endl;
      }
      cout << endl;

  }

  string Mesh::exportToOBJ(long totalVertices, long totalTextures, long totalNormals, int c){
    string myfile = "";

    myfile +=  "o " + meshName ;
    string str;
    if(c < 10) str = "00" + to_string(c);
    else {if (c < 100) str = "0" + to_string(c);
        else str = to_string(c);}
    myfile += str + '\n';
    for (auto v:vertices){
      auto x= v.X;
      myfile = myfile + "v "+to_string(x.x) +" " + to_string(x.y) + " " + to_string(x.z) + '\n';
    }
    myfile += textures;
    for (int t=0;t<triangles.size();t++){
        int x,y;
        if (trianglesTextures.size() > 0)
        x = trianglesTextures[t][0];
        else x = 0;
        if (trianglesNormals.size() >0)
        y = trianglesNormals[t][0];
        else y = 0;
        myfile+= "f " + to_string(triangles[t].A+1+totalVertices) +  "/"  + to_string(x+totalTextures) + "/" + to_string(y+totalNormals) + " ";
        if (trianglesTextures.size() > 0)
        x = trianglesTextures[t][1];
        else x = 0;
        if (trianglesNormals.size() >0)
        y = trianglesNormals[t][1];
        else y = 0;
        myfile+= to_string(triangles[t].B+1+totalVertices) +  "/" + to_string(x+totalTextures) + "/" + to_string(y+totalNormals) + " ";
        if (trianglesTextures.size() > 0)
        x = trianglesTextures[t][2];
        else x = 0;
        if (trianglesNormals.size() >0)
        y = trianglesNormals[t][2];
        else y = 0;
        myfile += to_string(triangles[t].C+1+totalVertices) + "/" + to_string(x+totalTextures) + "/" + to_string(y+totalNormals) + '\n';
    }
    return myfile;



  }
