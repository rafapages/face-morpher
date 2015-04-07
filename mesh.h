#ifndef MESH_H
#define MESH_H

#include <stdio.h>
//#include <iostream>
#include <fstream>
#include <vector>

#include "triangle.h"

class Mesh {

 public:
    Mesh();
    Mesh(const std::vector<Vector3f>& _vtx, const std::vector<Triangle> _tri);
    ~Mesh();

    // I/O
    void readOBJ(const std::string& _fileName);
    void writeOBJ(const std::string& _fileName);

    // Data access
    Vector3f getVector(unsigned int _index) const;
    Triangle getTriangle(unsigned int _index) const;
    void addVector(const Vector3f& _vector);
    void addTriangle(const Triangle& _triangle);

 private:
    std::vector<Vector3f> vtx_;
    std::vector<Triangle> tri_;
    unsigned int nVtx_, nTri_;

};

#endif // MESH_H
