#include "triangle.h"

Triangle::Triangle(unsigned int _i0, unsigned int _i1, unsigned int _i2){
    i_(0) = _i0;
    i_(1) = _i1;
    i_(2) = _i2;
}

Vector3i Triangle::getIndices() const {
    return i_;
}



