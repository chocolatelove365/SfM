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

void SfM(Eigen::Ref<const Eigen::MatrixXd> image0, Eigen::Ref<const Eigen::MatrixXd> image1, const int width, const int height, const double fov){
    const double foc = width / tan(fov / 180.0 * M_PI);
    cout << "foc: " << foc << "\n";
    
    Eigen::Matrix3d K;
    K <<
    foc, 0.0, 0.5 *(width - 1),
    0.0, foc, 0.5 *(height - 1),
    0.0, 0.0, 1.0;
    
    Matrix3d F = eight::fundamentalMatrix(image0, image1);
    cout << F << "\n";
    Matrix3d E = eight::essentialMatrix(K, F);
//    Matrix<double, 3, 4> pose0, pose1;
//    pose0 <<
//    1.0, 0.0, 0.0, 0.0,
//    0.0, 1.0, 0.0, 0.0,
//    0.0, 0.0, 1.0, 0.0;
    Matrix<double, 3, 4> pose1 = eight::pose(E, K, image0, image1);
//    cout << "distance: " << pose1.col(3).norm() << "\n";
//    cout << "pose0:\n" << pose0 << "\n";
//    cout << "pose1:\n" << pose1 << "\n";
//    Matrix<double, 3, 4> P0 = eight::cameraMatrix(K, pose0);
//    Matrix<double, 3, 4> P1 = eight::cameraMatrix(K, pose1);
//    Vector3d X = eight::triangulate(P0, P1, image0.col(7), image1.col(7));
//    cout << "X:\n" << X << "\n";
    Eigen::Matrix<double, 3, Eigen::Dynamic > pointsTriangulated = eight::structureFromTwoViews(K, pose1, image0, image1);
    double scale = 0.7 / (pointsTriangulated.col(0) - pointsTriangulated.col(1)).norm();
    cout << "scale: " << scale << "\n";
    pointsTriangulated.array() *=  scale;
    
    cout << pointsTriangulated << "\n";
}
