#include "morpher.h"

Morpher::Morpher()
{

}

Morpher::~Morpher()
{

}

void Morpher::readCameraFile(const std::string &_fileName){

    std::ifstream camFile(_fileName.c_str());

    if (camFile.is_open()){

        // First line contains the number of cameras
        std::string line;
        std::getline(camFile, line);
        sscanf(line.c_str(), "%u", &nCam_);

        // Now every camera calibration file is read
        for ( unsigned int i = 0; i < nCam_ ; i++){
            Camera c;
            std::getline(camFile,line);
            c.readCameraParameters(line);
            cameras_.push_back(c);
        }

    } else {
        std::cerr << "Unable to open " << _fileName << " file!" << std::endl;
        exit(-1);
    }

}

