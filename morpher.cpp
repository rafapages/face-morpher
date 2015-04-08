#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include "morpher.h"

// CGAL typedefs for 2D Delaunay triangulation
typedef CGAL::Exact_predicates_inexact_constructions_kernel                 Kernel;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel>   Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                            Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                         Delaunay;
typedef Kernel::Point_2                                                     Point;

Morpher::Morpher(){
    nCam_ = 0;
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
            faceControlPoints_.push_back(faceMesh_.getVertex(index));
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

    // Vector containing 2D positions of points in cam1
    // cam1 is the frontal view, so we will perform a
    // Delaunay triangulation of these points
    std::vector<Vector2f> cps1;

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
            cps1.push_back(v1);
        }

    } else {
        std::cerr << "Unable to open " << _fileName << " file!" << std::endl;
        exit(-1);
    }

    performDelaunayTri(cps1, controlTriangles_);

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

void Morpher::performDelaunayTri(const std::vector<Vector2f> &_vtx, std::vector<Triangle> &_tri) const{

    std::vector< std::pair<Point,unsigned int> > points;
    for (unsigned int i = 0; i < _vtx.size(); i++){
        const Vector2f current = _vtx[i];
        points.push_back(std::make_pair(Point(current[0], current[1]),i));
    }

    Delaunay triangulation;
    triangulation.insert(points.begin(),points.end());

    for(Delaunay::Finite_faces_iterator fit = triangulation.finite_faces_begin();
        fit != triangulation.finite_faces_end(); ++fit) {

        Delaunay::Face_handle face = fit;
        Vector3i newtri;
        for (unsigned int k = 0; k < 3; k++){
            newtri[k] = face->vertex(k)->info();
        }
//        const Triangle t(newtri[0], newtri[1], newtri[2]);
        const Triangle t(newtri[0], newtri[2], newtri[1]);
        _tri.push_back(t);

    }


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

Vector3f Morpher::getBarycenter(std::vector<Vector3f> _vtx) const {

    Vector3f barycenter(0.0,0.0,0.0);
//    for (unsigned int i = 0; i < _vtx.size(); i++){
//        barycenter += _vtx[i];
//    }

//    return barycenter / _vtx.size();

    barycenter = _vtx[0] + _vtx[8] + _vtx[5] + _vtx[6];
    barycenter = barycenter / 4;
    barycenter = barycenter + (barycenter - _vtx[23])*0.5;
    return barycenter;
}

void Morpher::setPyramids(){

    std::cerr << "Setting transformation pyramids... ";
    const Vector3f cpBar = getBarycenter(controlPoints_);
    const Vector3f faceBar = getBarycenter(faceControlPoints_);

    // As many pyramids as triangles previously calculated
    for (unsigned int i = 0; i < controlTriangles_.size(); i++){

        const Triangle t = controlTriangles_[i];
        Vector3f cpTriVertices[3];
        Vector3f faceTriVertices[3];
        for (unsigned int k = 0; k < 3; k++){
            cpTriVertices[k] = controlPoints_[t.getIndex(k)];
            faceTriVertices[k] = faceControlPoints_[t.getIndex(k)];
        }

//        std::cerr << "baseF: V0" << std::endl;
//        std::cerr << cpTriVertices[0] << " V1\n" << cpTriVertices[1] << " V2\n" << cpTriVertices[2] << std::endl;
        const Pyramid cpPyr(cpTriVertices[0], cpTriVertices[1], cpTriVertices[2], cpBar);
        controlPointPyramids_.push_back(cpPyr);

//        std::cerr << "baseF: V0" << std::endl;
//        std::cerr << faceTriVertices[0] << " V1\n" << faceTriVertices[1] << " V2\n" << faceTriVertices[2] << std::endl;
        const Pyramid facePyr(faceTriVertices[0], faceTriVertices[1], faceTriVertices[2], faceBar);
        facePyramids_.push_back(facePyr);
    }

    std::cerr << "done!\n";

}

void Morpher::transformFaceMesh(Mesh &_mesh) {


    // Transformation pyramids are set
    setPyramids();

    // A map to store new vertex indices
    std::map<unsigned int, int> indicesMap;
    for (unsigned int i = 0; i < faceMesh_.getNVtx(); i++){
        indicesMap[i] = -1;
    }

    // for each vertex of FaceMesh
    int newIndex = 0;
    for (unsigned int i = 0; i < faceMesh_.getNVtx(); i++){
        const Vector3f currV = faceMesh_.getVertex(i);
        Vector3f newV(0,0,0);

        // for each face pyramid
        for (unsigned int p = 0; p < facePyramids_.size(); p++){
            const Pyramid currFacePyr = facePyramids_[p];
            Vector3f duv(0,0,0);

            // if it belongs to the pyramid
            if (currFacePyr.getDUVparameters(currV, duv)){

                // We get its new coordinates
                const Pyramid currCPpyr = controlPointPyramids_[p];
                currCPpyr.get3DpointFromDUV(duv, newV);
                _mesh.addVector(newV);

                // We save the new value in our indices map
                indicesMap[i] = newIndex;
                newIndex++;
                break;
            }
        } // end per face pyramid

    } // end per vertex

    // for each triangle of FaceMesh we update its indices
    for (unsigned int i = 0; i < faceMesh_.getNTri(); i++){
        const Vector3i currInd = faceMesh_.getTriangle(i).getIndices();
        Vector3i newIndices;

        bool validTri = true;
        for (unsigned int k = 0; k < 3; k++){
            // In case it's not a transformed triangle
            if (indicesMap[currInd[k]] == -1){
                validTri = false;
                break;
            }
            newIndices[k] = indicesMap[currInd[k]];
        }
        // if it's not a valid triangle, we skip it
        if (!validTri) continue;
        Triangle newTri(newIndices[0], newIndices[1], newIndices[2]);
        _mesh.addTriangle(newTri);

    } // end per triangle

}

void Morpher::exportPyramidalMesh(const std::vector<Pyramid> &_pyrs) const{


    for (unsigned int i = 0; i < _pyrs.size(); i++){

        const Pyramid currP = _pyrs[i];
        Mesh m;
        Vector3f a,b,c;
        currP.getBase(a,b,c);
        m.addVector(a);
        m.addVector(b);
        m.addVector(c);
        m.addVector(currP.getBarycenter());

        Triangle t1(0,1,2);
        Triangle t2(0,2,3);
        Triangle t3(3,2,1);
        Triangle t4(0,3,1);
        m.addTriangle(t1);
        m.addTriangle(t2);
        m.addTriangle(t3);
        m.addTriangle(t4);

        std::stringstream s;
        s << "pyr" << i << ".obj";
        m.writeOBJ(s.str());
    }
}


