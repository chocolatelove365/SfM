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

Eigen::Matrix<double, 3, Eigen::Dynamic> SfM(Eigen::Ref<const Eigen::MatrixXd> image0, Eigen::Ref<const Eigen::MatrixXd> image1, const int width, const int height, const double fov0, const double fov1, Eigen::Matrix<double, 3, Eigen::Dynamic>& points3d, Eigen::Matrix<double, 3, 4>& pose1){
    const double foc0 = height * 0.5 / tan(fov0 / 360.0 * M_PI);
    const double foc1 = height * 0.5 / tan(fov1 / 360.0 * M_PI);
    
    Eigen::Matrix3d K0;
    K0 <<
    foc0, 0.0, 0.5 * (width - 1),
    0.0, foc0, 0.5 * (height - 1),
    0.0, 0.0, 1.0;
    
    Eigen::Matrix3d K1;
    K1 <<
    foc1, 0.0, 0.5 * (width - 1),
    0.0, foc1, 0.5 * (height - 1),
    0.0, 0.0, 1.0;
    
    Eigen::Matrix3d F = eight::fundamentalMatrix(image0, image1);
    Eigen::Matrix3d E = eight::essentialMatrix(K0, K1, F);
    pose1 = eight::pose(E, K0, K1, image0, image1);
    cout << "pose1:\n" << pose1 << "\n";
    points3d = eight::structureFromTwoViews(K0, K1, pose1, image0, image1);
    Eigen::Matrix<double, 3, 4> pose0 = Eigen::Matrix<double, 3, 4>::Identity();
    
//    for(int i = 0; i < points3d.cols(); i++){
//        points3d(1, i) = -points3d(1, i);
//        points3d(2, i) = -points3d(2, i);
//    }
    
    return points3d;
}
