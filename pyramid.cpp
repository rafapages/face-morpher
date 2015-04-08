#include "pyramid.h"

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

void Pyramid::getBase(Vector3f &_a, Vector3f &_b, Vector3f &_c) const{
    _a = p0_;
    _b = p1_;
    _c = p2_;
}

bool Pyramid::getDUVparameters(const Vector3f &_v, Vector3f &_duv) const{

    // Vertices facing back are discarded
    Vector3f n = getBaseNormal();
    if (n.dot(_v - barycenter_) <= 0){
        return false;
    }

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
    for (unsigned i = 0; i < 3; i++){
        M(i,0) = vbar(i);
    }
    for (unsigned i = 0; i < 3; i++){
        M(i,1) = p1p0(i);
    }
    for (unsigned i = 0; i < 3; i++){
        M(i,2) = p2p0(i);
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

void Pyramid::get3DpointFromDUV(const Vector3f &_duv, Vector3f &_v) const{

    const Vector3f C = p0_ + (p1_ - p0_) * _duv[1] + (p2_ - p0_) * _duv[2];
    _v = barycenter_ + _duv[0] * (C - barycenter_);

}

Vector3f Pyramid::getBaseNormal() const{
        Vector3f cp = (p1_ - p0_).cross(p2_ - p0_);
        return cp.normalized();
}




