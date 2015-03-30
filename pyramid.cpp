#include "pyramid.h"

Vector3f Pyramid::barycenter_;


Pyramid::Pyramid(){

}

Pyramid::Pyramid(const Vector3f& _a, const Vector3f& _b, const Vector3f& _c, const Vector3f& _bar){
    p0_ = _a;
    p1_ = _b;
    p2_ = _c;
    barycenter_ = _bar;
}

Pyramid::Pyramid(const Vector3f& _a, const Vector3f& _b, const Vector3f& _c){
    p0_ = _a;
    p1_ = _b;
    p2_ = _c;
}

Pyramid::~Pyramid(){

}

void Pyramid::setBarycenter(const Vector3f &_bar){
    barycenter_ = _bar;
}

Vector3f Pyramid::getBarycenter() const {
    return barycenter_;
}

// THIS METHOD STILL NEEDS TO BE TESTED
bool Pyramid::getDUVparameters(const Vector3f &_v, Vector3f &_duv){

    // In case _v is one og the vertices of the base:
    if (_v == p0_){
        _duv(0) = 1;
        _duv(1) = 0;
        _duv(0) = 0;
        return true;
    } else if (_v == p1_){
        _duv(0) = 1;
        _duv(1) = 1;
        _duv(0) = 0;
        return true;
    } else if (_v == p2_){
        _duv(0) = 1;
        _duv(1) = 0;
        _duv(0) = 1;
        return true;
    }

    // In any other case, we solve the line-plane intersection
    // problem in parametric form. Take a look at the description:
    // http://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
    const Vector3f p1p0 = p1_ - p0_;
    const Vector3f p2p0 = p2_ - p0_;
    const Vector3f vbar = _v - barycenter_;

    Matrix3f M, Minv;
    for (unsigned i = 0; i < 2; i++){
        M(0,i) = vbar(i);
    }
    for (unsigned i = 0; i < 2; i++){
        M(1,i) = p1p0(i);
    }
    for (unsigned i = 0; i < 2; i++){
        M(2,i) = p2p0(i);
    }

    Minv = M.inverse();
    Vector3f tuv = Minv*(_v - p0_);
    const float t = tuv(0);
    const float u = tuv(1);
    const float v = tuv(2);

    Vector3f pint = _v + (barycenter_ - _v) * t; // Intersection of line and plane
    const float dab = (_v - barycenter_).norm();
    const float dbpint = (pint - barycenter_).norm();

    const float delta = dab / dbpint;
    _duv(0) = delta;
    _duv(1) = u;
    _duv(2) = v;

    // _v belongs to the pyramid if u,v € [0,1] and u+1 <= 1
    if ( (0 <= u) && (u <= 1) && (0 <= v) && (v <= 1) && (u+v <= 1) ){
        return true;
    } else {
        return false;
    }

}




