/* 
 *  Copyright (c) 2014  Rafael Pagés (rps (at) gti.ssr.upm.es)
 *    and Universidad Politécnica de Madrid
 *
 *  This file is part of faceMorpher
 *
 *  Multitex is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Multitex is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <iostream>
#include "morpher.h"

using namespace std;

int main(int argc, char *argv[]){

    if (argc != 6){
        std::cerr << "Wrong number of input parameters!" << std::endl;
        std::cerr << "Usage: " << argv[0] << " <faceMesh.obj> <cameraCalibrationFile> <imageListFile> <controlPointCorrespondances> <controlPointIndices>" << std::endl;
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
    std::string meshname(argv[1]);
    meshname = meshname.substr(0, meshname.size()-4);
    meshname += "_transformed.obj";
    m.writeOBJ(meshname);


    return 0;
}


