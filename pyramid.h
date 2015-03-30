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

    // Computes the delta-u-v parameters of
    // a vertex with respect to the pyramid.
    // Returns false if the vertex is not
    // inside the pyramid.
    bool getDUVparameters(const Vector3f& _v, Vector3f& _uvd);


 private:

    static Vector3f barycenter_;
    Vector3f p0_, p1_, p2_;

};

#endif // PYRAMID_H
