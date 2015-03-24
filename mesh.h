#ifndef MESH_H
#define MESH_H

#include <eigen3/Eigen/Dense>
#include <stdio.h>

class Triangle;

using namespace Eigen;

class Mesh {

public:
    Mesh();
    ~Mesh();

    // I/O
    void readOBJ(std::string _fileName);
    void writeOBJ(std::string _fileName);


    std::vector<Vector3f> vtx_;
    std::vector<Triangle> tri_;

};

#endif // MESH_H
