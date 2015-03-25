#include "mesh.h"

Mesh::Mesh(){
    nVtx_ = nTri_ = 0;
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
//                vtx_.push_back(Vector3f(a,b,c));
                this->addVector(Vector3f(a,b,c));
                nVtx_++;
            } else if (initline.compare("f ") == 0) {
                unsigned int a, b, c;
                sscanf(line.c_str(), "f %d %d %d", &a, &b, &c);
//                tri_.push_back(Triangle(a,b,c));
                this->addTriangle(Triangle(a,b,c));
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
}

void Mesh::writeOBJ(const std::string& _fileName){

}

void Mesh::addVector(const Vector3f& _vector){
    vtx_.push_back(_vector);
}

void Mesh::addTriangle(const Triangle& _triangle){
    tri_.push_back(_triangle);
}



