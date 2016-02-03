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

#ifndef MORPHER_H
#define MORPHER_H

#include <map>

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
    int getCameraIndex(const std::string& _imageName) const;
    void getControlPoints(std::vector<Vector3f> & _cps) const;

    // Transforms faceMesh into a new mesh (_mesh) using
    // the information from the subjects control points
    void transformFaceMesh(Mesh& _mesh);

    // Stupid method to test the correct pyramid forming:
    // Export pyramidalset of meshes
    void exportPyramidalMesh(const std::vector<Pyramid>& _pyrs) const;

 private:

    // 2D Delaunay triangulation
    // input: vector with 2D vertices
    // output: vector with triangle indices stored in a Vector3i (int)
    void performDelaunayTri(const std::vector<Vector2f>& _vtx, std::vector<Triangle>& _tri) const;

    // Get 3D position of a point given
    // its pixel location in two images
    Vector3f triangulatePoint(int _cam1index, const Vector2f& _pix1, int _cam2index, const Vector2f& _pix2) const;

    // Returns a map with controlPoints_ sorted
    // by distance to the specified camera, but not
    // the camera position, but its orientation
    void sortControlPointsFromCam(unsigned int _camIndex, std::multimap<float, unsigned int>& _distances) const;

    // Calculates the barycenter of a set of points
    Vector3f getBarycenter(std::vector<Vector3f> _vtx) const;

    // Transformation pyramids are set
    void setPyramids();



    Mesh faceMesh_;
    unsigned int nCam_;
    std::vector<Camera> cameras_;
    std::vector<std::string> imageList_;
    unsigned int frontCamera_;

    // Two sets of pyramids needed for
    // the transformation
    std::vector<Pyramid> controlPointPyramids_;
    std::vector<Pyramid> facePyramids_;

    // Control points triangulated from images
    std::vector<Vector3f> controlPoints_;
    // Control points in faceMesh_
    std::vector<Vector3f> faceControlPoints_;

    // Indices to the triangulated control points:
    // these triangles will define the base of every pyramid
    // depending if they are used with control points in
    // subjects face or in the face model.
    std::vector<Triangle> controlTriangles_;

};

#endif // MORPHER_H
