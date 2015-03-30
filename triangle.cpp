#include "triangle.h"

Triangle::Triangle(){

}

Triangle::Triangle(unsigned int _i0, unsigned int _i1, unsigned int _i2){
    i_(0) = _i0;
    i_(1) = _i1;
    i_(2) = _i2;
}

Vector3i Triangle::getIndices() const {
    return i_;
}

int Triangle::getIndex(unsigned int _index) const {
    if (_index > 2){
        std::cerr << "Wrong index!" << std::endl;
        exit(-1);
    }

    return i_(_index);
}

//Vector2f Triangle::getUVcoordinates(const Vector3f &_v) const {
//}

//bool Triangle::isInside(const Vector3f &_v) const {
//}


