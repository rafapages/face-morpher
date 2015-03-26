#include "morpher.h"

Morpher::Morpher(){

}

Morpher::~Morpher(){

}

void Morpher::readCameraFile(const std::string &_fileName){

    std::ifstream camFile(_fileName.c_str());

    if (camFile.is_open()){

        // First line contains the number of cameras
        std::string line;
        std::getline(camFile, line);
        sscanf(line.c_str(), "%u", &nCam_);

        std::cerr << "Reading " << nCam_ << " camera parameters... ";

        // Now every camera calibration file is read
        for ( unsigned int i = 0; i < nCam_ ; i++){
            Camera c;
            std::getline(camFile,line);
            c.loadCameraParameters(line);
            cameras_.push_back(c);
        }

        std::cerr << "done!\n";

    } else {
        std::cerr << "Unable to open " << _fileName << " file!" << std::endl;
        exit(-1);
    }

}

void Morpher::setFaceMesh(const Mesh &_faceMesh){
    faceMesh_ = _faceMesh;
}

void Morpher::setFaceMesh(const std::string& _fileName){
    Mesh m;
    m.readOBJ(_fileName);
    faceMesh_ = m;
}

Camera Morpher::getCamera(unsigned int _index){
    return cameras_[_index];
}

