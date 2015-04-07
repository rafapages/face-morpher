#ifndef MORPHER_H
#define MORPHER_H

#include "camera.h"
#include "pyramid.h"

class Morpher {
 public:
    Morpher();
    ~Morpher();

    // I/O
    void readCameraFile(const std::string& _fileName);
    void readImageList(const std::string& _fileName);
    void readCPindicesFile(const std::string& _fileName);
    void readCPcorrespondances(const std::string& _fileName);

    // Access data
    void setFaceMesh(const Mesh& _faceMesh);
    void setFaceMesh(const std::string& _fileName);
    Camera getCamera(unsigned int _index) const;
    Pyramid getPyramid(unsigned int _index) const;
    int getCameraIndex(const std::string& _imageName) const;
    void getControlPoints(std::vector<Vector3f> & _cps) const;

    // 2D Delaunay triangulation
    // input: vector with 2D vertices
    // output: vector with triangle indices stored in a Vector3i (int)
    void performDelaunayTri(const std::vector<Vector2f>& _vtx, std::vector<Vector3i>& _tri);

    // Get 3D position of a point given
    // its pixel location in two images
    Vector3f triangulatePoint(int _cam1index, const Vector2f& _pix1, int _cam2index, const Vector2f& _pix2) const;

 private:

    Mesh faceMesh_;
    unsigned int nCam_;
    std::vector<Camera> cameras_;
    std::vector<std::string> imageList_;
    std::vector<Pyramid> pyramids_;
    // control points triangulated from images
    std::vector<Vector3f> controlPoints_;
    // indices to control points in faceMesh_
    std::vector<int> faceCPindices_;

};

#endif // MORPHER_H
