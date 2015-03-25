#ifndef MORPHER_H
#define MORPHER_H

#include "camera.h"

class Morpher {
 public:
    Morpher();
    ~Morpher();

    // I/O
    void readCameraFile(const std::string& _fileName);

    // Access data
    void setFaceMesh(const Mesh& _faceMesh);

 private:

    Mesh faceMesh_;
    unsigned int nCam_;
    std::vector<Camera> cameras_;

};

#endif // MORPHER_H
