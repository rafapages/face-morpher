# 3D Face Morpher

This project transforms a generic 3D facial model into another one using a person’s facial features. This system is part of the work described in the following paper:

[Face Lift Surgery for Reconstructed Virtual Humans] (http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=6079374&tag=1)

## Usage
The system takes the following input parameters:

*  <faceMesh.obj> A 3D model of a generic face.
*  <cameraCalibrationFile> Calibration matrices of the cameras used.
*  <imageListFile> List of images used for triangulating facial features.
*  <controlPointCorrespondances> List of correspondances in the input images 
*  <controlPointIndices> Indices to the same previous feature points in the generic facial mesh

## Citacion
If you use this software for scientific purposes, please don’t forget to cite us! Thanks!

@inproceedings{pages2011face,  
author={Pag\’{e}s, R. and Arnaldo, S. and Mor\’{a}n, F.},  
booktitle={Cyberworlds (CW), 2011 International Conference on},  
title={Face Lift Surgery for Reconstructed Virtual Humans},  
year={2011},  
month={Oct},  
pages={249-253},  
doi={10.1109/CW.2011.13},}  

