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

    std::ifstream corrFile(_fileName.c_str());

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
    int index = -1;
    for (unsigned int i = 0; i < imageList_.size(); i++){
        if (_image.compare(imageList_[i]) == 0){
            index = i;
            break;
        }
    }

    return index;
}

void Morpher::getControlPoints(std::vector<Vector3f> &_cps) const{
    _cps = controlPoints_;
}

Vector3f Morpher::triangulatePoint(int _cam1index, const Vector2f &_pix1, int _cam2index, const Vector2f &_pix2) const {

    // Cameras projection matrices
    MatrixXf P1,P2;
    P1 = cameras_[_cam1index].getProjectiveMatrix();
    P2 = cameras_[_cam2index].getProjectiveMatrix();

    // The matrix of the system AX=0
    MatrixXf A(4,4);
    A.row(0) = _pix1(0) * P1.row(2) - P1.row(0);
    A.row(1) = _pix1(1) * P1.row(2) - P1.row(1);
    A.row(2) = _pix2(0) * P2.row(2) - P2.row(0);
    A.row(3) = _pix2(1) * P2.row(2) - P2.row(1);

    // System is solved using Lagrange multipliers
    // http://stackoverflow.com/questions/19947772/non-trival-solution-for-ax-0-using-eigen-c
    Matrix4f M = A.adjoint() * A;
    const Vector4f point3Dh = SelfAdjointEigenSolver<Matrix4f>(M).eigenvectors().col(0);
    // Vertex from homogeneous coordinates
    const Vector3f point3D (point3Dh(0)/point3Dh(3), point3Dh(1)/point3Dh(3), point3Dh(2)/point3Dh(3));
    return point3D;

}

