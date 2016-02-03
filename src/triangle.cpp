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
