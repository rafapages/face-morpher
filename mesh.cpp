#include "mesh.h"

Mesh::Mesh(){
    nVtx_ = nTri_ = 0;
}

Mesh::Mesh(const std::vector<Vector3f> &_vtx, const std::vector<Triangle> _tri){
    vtx_ = _vtx;
    tri_ = _tri;
    nVtx_ = _vtx.size();
    nTri_ = _tri.size();
}

Mesh::~Mesh(){

}

void Mesh::readOBJ(const std::string& _fileName){

    std::ifstream meshFile(_fileName.c_str());

    if (meshFile.is_open()){

        std::string line;
        std::string initline;

        // First avoid comments
        do {
            std::getline(meshFile, line);
            initline = line.substr(0,2);
        } while ((initline.compare("v ") != 0) && (initline.compare("f ") != 0));

        while (!meshFile.eof()){
            if (initline.compare("v ") == 0){
                float a, b, c;
                sscanf(line.c_str(), "v %f %f %f", &a, &b, &c);
                this->addVector(Vector3f(a,b,c));
                nVtx_++;
            } else if (initline.compare("f ") == 0) {
                unsigned int a, b, c;
                sscanf(line.c_str(), "f %d %d %d", &a, &b, &c);
                this->addTriangle(Triangle(a-1,b-1,c-1)); // OBJ indices start at 1
                nTri_++;
            } else {
                // Do nothing by now...
            }
            std::getline(meshFile, line);
            initline = line.substr(0,2);
        }

    } else {
        std::cerr << "Unable to read " << _fileName << " file!" << std::endl;
        exit(-1);
    }

    meshFile.close();
}

void Mesh::writeOBJ(const std::string& _fileName){

    std::cerr << "Exporting mesh " << _fileName << std::endl;

    std::ofstream outMesh(_fileName.c_str());

    // I should first print a header...

    // Vertices
    for (unsigned int i = 0; i < nVtx_; i++){
        const Vector3f current = vtx_[i];
        outMesh << "v";
        for (unsigned int j = 0; j < 3; j++){
            outMesh << " " << current(j);
        }
        outMesh << "\n";
    }

    for (unsigned int i = 0; i < nTri_; i++){
        const Vector3i current = tri_[i].getIndices();
        outMesh << "f";
        for (unsigned int j = 0; j < 3; j++){
            outMesh << " " << current(j)+1; // OBJ indices start at 1
        }
        outMesh << "\n";
    }

    outMesh.close();
}

void Mesh::addVector(const Vector3f& _vector){
    vtx_.push_back(_vector);
}

void Mesh::addTriangle(const Triangle& _triangle){
    tri_.push_back(_triangle);
}

Vector3f Mesh::getVertex(unsigned int _index) const {
    return vtx_[_index];
}

Triangle Mesh::getTriangle(unsigned int _index) const {
    return tri_[_index];
}



