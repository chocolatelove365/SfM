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

Eigen::Matrix<double, 3, Eigen::Dynamic> SfM(Eigen::Ref<const Eigen::MatrixXd> image0, Eigen::Ref<const Eigen::MatrixXd> image1, const int width, const int height, const double fov){
    const double foc = height * 0.5 / tan(fov / 360.0 * M_PI);
    cout << "foc: " << foc << "\n";
    
    Eigen::Matrix3d K;
    K <<
    foc, 0.0, 0.5 *(width - 1),
    0.0, foc, 0.5 *(height - 1),
    0.0, 0.0, 1.0;
    
    Matrix3d F = eight::fundamentalMatrix(image0, image1);
    cout << "F:\n" << F << "\n";
    Matrix3d E = eight::essentialMatrix(K, F);
    Matrix<double, 3, 4> pose1 = eight::pose(E, K, image0, image1);
    cout << "pose1:\n" << pose1 << "\n";
    Matrix4d pose2;
    pose2 << pose1, 0, 0, 0, 1;
    cout << "pose2_inv:\n" << pose2.inverse() << "\n";
    Eigen::Matrix<double, 3, Eigen::Dynamic> points3d = eight::structureFromTwoViews(K, pose1, image0, image1);
    for(int i = 0; i < points3d.cols(); i++){
        points3d(0, i) = -points3d(0, i);
//        points3d(1, i) = -points3d(1, i);
//        points3d(2, i) = -points3d(2, i);
    }
    double scale = 1.0 / (points3d.col(0) - points3d.col(1)).norm();
    cout << "scale: " << scale << "\n";
    points3d.array() *=  scale;
    return points3d;
}
