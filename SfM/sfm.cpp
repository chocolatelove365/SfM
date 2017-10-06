//
//  sfm.cpp
//  SfM
//
//  Created by tomiya on 2017/10/06.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "sfm.hpp"
#include "eight/fundamental.h"
#include "eight/essential.h"
#include "eight/triangulate.h"
#include "eight/project.h"
#include "eight/structure.h"

using namespace std;
using namespace Eigen;

void SfM(){
    const int width = 1920;
    const int height = 1080;
    const double foc = width / tan(4.375 / 90.0 * M_PI);
    const int n_points = 8;
    cout << "foc: " << foc << "\n";
    
    Eigen::Matrix3d K;
    K <<
    foc, 0.0, 0.5 *(width - 1),
    0.0, foc, 0.5 *(height - 1),
    0.0, 0.0, 1.0;
    
    Matrix<double, 2, n_points> imageL, imageR;
    imageL <<
    1465, 1378, 589, 471, 380, 467, 1215, 1326,
    776, 836, 919, 878, 484, 428, 357, 399;
    imageR <<
    1400, 1288, 518, 434, 553, 659, 1392, 1477,
    772, 817, 764, 707, 328, 289, 337, 392;
    
    Matrix3d F = eight::fundamentalMatrix(imageL, imageR);
    cout << F << "\n";
    Matrix3d E = eight::essentialMatrix(K, F);
    Matrix<double, 3, 4> poseL, poseR;
    poseL <<
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0;
    poseR = eight::pose(E, K, imageL, imageR);
    cout << "distance: " << poseR.col(3).norm() << "\n";
    cout << "PoseL:\n" << poseL << "\n";
    cout << "PoseR:\n" << poseR << "\n";
    Matrix<double, 3, 4> P_L = eight::cameraMatrix(K, poseL);
    Matrix<double, 3, 4> P_R = eight::cameraMatrix(K, poseR);
    Vector3d X = eight::triangulate(P_L, P_R, imageL.col(7), imageR.col(7));
    cout << "X:\n" << X << "\n";
    Eigen::Matrix<double, 3, Eigen::Dynamic > pointsTriangulated = eight::structureFromTwoViews(K, poseR, imageL, imageR);
    double scale = 0.7 / (pointsTriangulated.col(0) - pointsTriangulated.col(1)).norm();
    cout << "scale: " << scale << "\n";
    pointsTriangulated.array() *=  scale;
    
    cout << pointsTriangulated << "\n";
}
