#ifndef MORPHER_H
#define MORPHER_H

#include "camera.h"
#include "pyramid.h"

class Morpher {
 public:
    Morpher();
    ~Morpher();

    // I/O
    void readCameraFile(const std::string& _fileName);

    // Access data
    void setFaceMesh(const Mesh& _faceMesh);
    void setFaceMesh(const std::string& _fileName);
    Camera getCamera(unsigned int _index);

 private:

    Mesh faceMesh_;
    unsigned int nCam_;
    std::vector<Camera> cameras_;

};

#endif // MORPHER_H
