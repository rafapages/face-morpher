#ifndef CAMERA_H
#define CAMERA_H

#include "mesh.h"

class Camera{

 public:
    Camera();
    ~Camera();

    void readCameraParameters(const std::string& _textline);

 private:

    Matrix3f R_;
    Matrix3f K_;
    Vector3f position_;
    unsigned int imWidth, imHeight;




};

#endif // CAMERA_H
