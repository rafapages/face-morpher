#include <iostream>
#include "morpher.h"

using namespace std;

int main(int argc, char *argv[]){

    if (argc != 6){
        std::cerr << "Wrong number of input parameters!" << std::endl;
        std::cerr << "Usage: " << argv[0] << "<faceMesh.obj> <cameraCalibrationFile> <imageListFile> <controlPointCorrespondances> <controlPointIndices>" << std::endl;
        return -1;
    }

    // Reading input files
    Mesh faceMesh;
    faceMesh.readOBJ(argv[1]);

    Morpher morpher;
    morpher.setFaceMesh(faceMesh);

    morpher.readCameraFile(argv[2]);
    morpher.readImageList(argv[3]);
    morpher.readCPcorrespondances(argv[4]);
    morpher.readCPindicesFile(argv[5]);

    // Perform the transformation
    Mesh m;
    morpher.transformFaceMesh(m);
    m.writeOBJ("transformada.obj");


    return 0;
}


