#ifndef CAMERA_H
#define CAMERA_H

#include <eigen3/Eigen/Dense>

class Camera{

 public:
    Camera();
    ~Camera();

 private:

    Eigen::Matrix3f R_;
    Eigen::Matrix3f K_;
    Eigen::Vector3f position_;
    unsigned int imWidth, imHeight;




};

#endif // CAMERA_H
