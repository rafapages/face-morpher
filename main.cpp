#include <iostream>

//// CGAL stuff
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Delaunay_triangulation_3.h>
//#include <CGAL/Triangulation_vertex_base_with_info_3.h>

#include "morpher.h"

//// CGAL typedefs
//typedef CGAL::Exact_predicates_inexact_constructions_kernel               Kernel;
//typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned int, Kernel> Vb;
//typedef CGAL::Triangulation_data_structure_3<Vb>                          Tds;
//typedef CGAL::Delaunay_triangulation_3<Kernel, Tds>                       Delaunay;
//typedef Delaunay::Point Point;

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

//    // Delaunay test
//    std::vector<Vector3f> cps;
//    morpher.getControlPoints(cps);

//    std::ofstream delau("delautest.obj");
//    for (unsigned int i = 0; i < cps.size(); i++){
//        const Vector3f current = cps[i];
//        delau << "v " << current[0] << " " << current[1] << " " << current[2] << std::endl;
//    }

//    std::vector< std::pair<Point,unsigned int> > points;
//    for (unsigned int i = 0; i < cps.size(); i++){
//        const Vector3f current = cps[i];
//        points.push_back(std::make_pair(Point(current[0], current[1], current[2]), i));
//    }

//    Delaunay triangulation;
//    triangulation.insert(points.begin(),points.end());

//    Delaunay::Finite_facets_iterator fit = triangulation.finite_facets_begin();
//    for (; fit != triangulation.finite_facets_end(); ++fit){
//        delau << "f";
//       for (unsigned int k = 1; k <=3 ; k++){
//           delau << " " << fit->first->vertex( (fit->second+k)%4 )->info() + 1;
//       }
//       delau << "\n";
//    }

//    delau.close();

    return 0;
}


