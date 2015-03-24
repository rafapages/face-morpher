#ifndef MESH_H
#define MESH_H

#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>

#include "triangle.h"

using namespace Eigen;

class Mesh {

 public:
    Mesh();
    ~Mesh();

    // I/O
    void readOBJ(const std::string _fileName);
    void writeOBJ(const std::string _fileName);

    // Data access
    Vector3f getVector(const unsigned int _index);
    Triangle getTriangle(const unsigned int _index);
    void addVector(const Vector3f _vector);
    void addTriangle(const Triangle _vector);

 private:
    std::vector<Vector3f> vtx_;
    std::vector<Triangle> tri_;

};

#endif // MESH_H
