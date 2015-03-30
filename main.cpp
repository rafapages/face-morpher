#include <iostream>

#include "morpher.h"

using namespace std;

int main(int argc, char *argv[]){


    Morpher morpher;
    morpher.readCameraFile(argv[1]);
    morpher.readImageList(argv[2]);
    morpher.readCPcorrespondances(argv[3]);
//    morpher.readCPindicesFile(argv[2]);

//    Mesh m;
//    m.readOBJ(argv[2]);

//    morpher.setFaceMesh(m);

//    m.writeOBJ("test.obj");

    return 0;
}


