#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace Eigen;

class Triangle{

 public:

    Triangle();
    Triangle(unsigned int _i0, unsigned int _i1, unsigned int _i2);

    // Data access
    Vector3i getIndices() const;
    int getIndex(unsigned int _index) const; // 0, 1 or 2

private:
    Vector3i i_;

};

#endif // TRIANGLE_H
