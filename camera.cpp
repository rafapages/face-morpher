#include "camera.h"

Camera::Camera()
{

}

Camera::~Camera()
{

}

void Camera::readCameraParameters(const std::string &_textline){

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



