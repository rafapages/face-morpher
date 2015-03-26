#include "camera.h"

Camera::Camera(){
    imWidth = imHeight = 0;
}

Camera::~Camera(){

}

void Camera::loadCameraParameters(const std::string &_textline){

    std::stringstream line(_textline);

    // Intrinsic parameters
    for (unsigned int i = 0; i < 3; i++){
        for (unsigned int j = 0; j < 3; j++){
            float value;
            line >> value;
            this->K_(i,j) = value;
        }
    }

    // Extrinsic parameteres
    for (unsigned int i = 0; i < 3; i++){
        for (unsigned int j = 0; j < 3; j++){
            float value;
            line >> value;
            this->R_(i,j) = value;
        }
    }

    // Camera position
    for (unsigned int i = 0; i < 3; i++){
        float value;
        line >> value;
        this->position_(i) = value;
    }

    // Image dimensions
    unsigned int uivalue;
    line >> uivalue;
    this->imWidth = uivalue;
    line >> uivalue;
    this->imHeight = uivalue;

//    std::cerr << "K matrix: " << std::endl;
//    std::cerr << this->K_ << std::endl;
//    std::cerr << "R matrix: " << std::endl;
//    std::cerr << this->R_ << std::endl;
//    std::cerr << "position: " << std::endl;
//    std::cerr << this->position_ << std::endl;
//    std::cerr << "Image: " << this->imWidth << "x" << this->imHeight << std::endl;

}

Matrix3f Camera::getIntrinsicParam() const {
    return K_;
}

Matrix3f Camera::getExtrinsicParam() const {
    return R_;
}

Vector3f Camera::getPosition() const {
    return position_;
}

Vector2i Camera::getImageDim() const {
    const Vector2i dim(imWidth, imHeight);
    return dim;
}

Vector3f Camera::transform2CameraCoord(const Vector3f &_v) const {

    const Vector3f aux = _v - position_;
    return R_ * aux;

}

Vector3f Camera::transform2TextureCoord(const Vector3f &_v) const {

    const Vector3f aux = transform2CameraCoord(_v);
    return K_ * aux;

}

Vector2f Camera::transform2uvCoord(const Vector3f &_v) const {

    const Vector3f aux = transform2TextureCoord(_v);
    // u = p.x/p.z
    // v = p.y/p.z
    return Vector2f(aux(0)/aux(2), aux(1)/aux(2));
}



