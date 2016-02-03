/* 
 *  Copyright (c) 2014  Rafael Pagés (rps (at) gti.ssr.upm.es)
 *    and Universidad Politécnica de Madrid
 *
 *  This file is part of faceMorpher
 *
 *  Multitex is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Multitex is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

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
