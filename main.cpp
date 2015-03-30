#include <iostream>

#include "morpher.h"

using namespace std;

int main(int argc, char *argv[]){

    if (argc != 5){
        std::cerr << "Wrong number of input parameters!" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <cameraCalibrationFile> <imageListFile> <controlPointCorrespondances> <controlPointIndices>" << std::endl;
        return -1;
    }

    Morpher morpher;
    morpher.readCameraFile(argv[1]);
    morpher.readImageList(argv[2]);
    morpher.readCPcorrespondances(argv[3]);
    morpher.readCPindicesFile(argv[4]);

//    Mesh m;
//    m.readOBJ(argv[2]);

//    morpher.setFaceMesh(m);

//    m.writeOBJ("test.obj");

    return 0;
}


