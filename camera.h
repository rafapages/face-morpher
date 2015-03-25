#ifndef CAMERA_H
#define CAMERA_H

#include "mesh.h"

class Camera{

 public:
    Camera();
    ~Camera();

    // Read parameters from text line
    void loadCameraParameters(const std::string& _textline);

    // Data access
    Matrix3f getIntrinsicParam() const;
    Matrix3f getExtrinsicParam() const;
    Vector3f getPosition() const;
    Vector2i getImageDim() const;

 private:

    Matrix3f K_; // Intrinsic parameters
    Matrix3f R_; // Extrinsic parameters
    Vector3f position_;
    unsigned int imWidth, imHeight;

};

#endif // CAMERA_H
