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

    camFile.close();
}

void Morpher::readImageList(const std::string &_fileName){

    std::cerr << "Reading image list file...";

    std::ifstream listFile(_fileName.c_str());

    if (listFile.is_open()){

        std::string line;
        while (!listFile.eof()){
            std::getline(listFile, line);
            std::cerr << line << std::endl;
            imageList_.push_back(line);
        }

    } else {
        std::cerr << "Unable to open " << _fileName << " file!" << std::endl;
        exit(-1);
    }

    std::cerr << " done!\n";

    listFile.close();
}

void Morpher::readCPindicesFile(const std::string &_fileName){

    std::cerr << "Reading control point index file...";

    std::ifstream indexFile(_fileName.c_str());

    if (indexFile.is_open()){

        std::string line;
        int index;
        while (!indexFile.eof()){
            std::getline(indexFile, line);
            std::stringstream ss;
            ss << line;
            ss >> index;
            faceCPindices_.push_back(index);
        }

    } else {
        std::cerr << "Unable to open " << _fileName << " file!" << std::endl;
        exit(-1);
    }

    std::cerr << " done!\n";
    indexFile.close();
}

void Morpher::readCPcorrespondances(const std::string &_fileName){

    std::cerr << "Reading control point correspondaces file...";

    std::ifstream corrFile(_fileName);

    if (corrFile.is_open()){

        std::string line;
        while (!corrFile.eof()){
            std::getline(corrFile, line);
            std::stringstream ss(line);
            std::string cam1;
            ss >> cam1;
            float pixx, pixy;
            ss >> pixx;
            ss >> pixy;
            Vector2f v1(pixx, pixy);
            std::string cam2;
            ss >> cam2;
            ss >> pixx;
            ss >> pixy;
            Vector2f v2(pixx, pixy);
            Vector3f point = triangulatePoint(getCameraIndex(cam1), v1, getCameraIndex(cam2), v2);
            controlPoints_.push_back(point);
        }

    } else {
        std::cerr << "Unable to open " << _fileName << " file!" << std::endl;
        exit(-1);
    }

    std::cerr << " done!\n";
    corrFile.close();
}


void Morpher::setFaceMesh(const Mesh &_faceMesh){
    faceMesh_ = _faceMesh;
}

void Morpher::setFaceMesh(const std::string& _fileName){
    Mesh m;
    m.readOBJ(_fileName);
    faceMesh_ = m;
}

Camera Morpher::getCamera(unsigned int _index) const {
    return cameras_[_index];
}

Pyramid Morpher::getPyramid(unsigned int _index) const {
    return pyramids_[_index];
}

int Morpher::getCameraIndex(const std::string& _image) const{
    int index;
    for (unsigned int i = 0; i < imageList_.size(); i++){
        if (_image.compare(imageList_[i]) == 0){
            index = i;
            break;
        }
    }

    return index;
}

Vector3f Morpher::triangulatePoint(int _cam1index, const Vector2f &_pix1, int _cam2index, const Vector2f &_pix2) const {

    // To be filled...

    return Vector3f(0,0,0);
}

