#include <iostream>

#include "morpher.h"

using namespace std;

int main(int argc, char *argv[]){

    Morpher morpher;
    morpher.readCameraFile(argv[1]);

    Mesh m;
    m.readOBJ(argv[2]);
    m.writeOBJ("test.obj");

    return 0;
}

