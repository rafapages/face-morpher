#ifndef MESH_H
#define MESH_H

#include <stdio.h>
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
    Vector3f getVertex(unsigned int _index) const;
    Triangle getTriangle(unsigned int _index) const;
    void addVector(const Vector3f& _vector);
    void addTriangle(const Triangle& _triangle);
    unsigned int getNVtx() const;
    unsigned int getNTri() const;

    // Triangle normal, normalized
    Vector3f getTriangleNormal(unsigned int _index) const;
    Vector3f getTriangleNormal(const Vector3f& _a, const Vector3f& _b, const Vector3f& _c) const;

 private:
    std::vector<Vector3f> vtx_;
    std::vector<Triangle> tri_;
    unsigned int nVtx_, nTri_;

};

#endif // MESH_H
