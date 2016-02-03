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
 
#ifndef PYRAMID_H
#define PYRAMID_H

#include "triangle.h"

class Pyramid{

 public:
    Pyramid();
    Pyramid(const Vector3f& _a, const Vector3f& _b, const Vector3f& _c, const Vector3f& _bar);
    Pyramid(const Vector3f& _a, const Vector3f& _b, const Vector3f& _c);
    ~Pyramid();

    // Data access
    void setBarycenter(const Vector3f& _bar);
    Vector3f getBarycenter() const;
    void getBase(Vector3f& _a, Vector3f& _b, Vector3f& _c) const;

    // Computes the delta-u-v parameters of
    // a vertex with respect to the pyramid.
    // Returns false if the vertex is not
    // inside the pyramid.
    bool getDUVparameters(const Vector3f& _v, Vector3f& _duv) const;

    // Calculates the position of a 3D point, given
    // its delta-u-v parameters
    void get3DpointFromDUV(const Vector3f& _duv, Vector3f& _v) const;

    // Normal of the base, normalized
    Vector3f getBaseNormal() const;


 private:

    Vector3f barycenter_;
    Vector3f p0_, p1_, p2_;

};

#endif // PYRAMID_H
