#include <iostream>
#include "morpher.h"

using namespace std;

int main(int argc, char *argv[]){

    if (argc != 6){
        std::cerr << "Wrong number of input parameters!" << std::endl;
        std::cerr << "Usage: " << argv[0] << "<faceMesh.obj> <cameraCalibrationFile> <imageListFile> <controlPointCorrespondances> <controlPointIndices>" << std::endl;
        return -1;
    }

//    std::stringstream s;
//    s << "hola" << 2 << ".ply";
//    std::cerr << s.str() << std::endl;

    Vector3f a(1,2,3);
    std::cerr << a[0] << " es " << a(0) << std::endl;
    a[1] = 47;
    std::cerr << a[1] << " es " << a(1) << std::endl;
    a(2) = 4.5;
    std::cerr << a[2] << " es " << a(2) << std::endl;


    // Reading input files
    Mesh faceMesh;
    faceMesh.readOBJ(argv[1]);

    Morpher morpher;
    morpher.setFaceMesh(faceMesh);

    morpher.readCameraFile(argv[2]);
    morpher.readImageList(argv[3]);
    morpher.readCPcorrespondances(argv[4]);
    morpher.readCPindicesFile(argv[5]);

    //
    Mesh m;
    morpher.transformFaceMesh(m);
    m.writeOBJ("transformada.obj");


    return 0;
}


