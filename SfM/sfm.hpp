//
//  sfm.hpp
//  SfM
//
//  Created by tomiya on 2017/10/06.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#ifndef sfm_hpp
#define sfm_hpp

#include <stdio.h>

//Eigen::Matrix<double, 3, Eigen::Dynamic> SfM(Eigen::Ref<const Eigen::MatrixXd> image0, Eigen::Ref<const Eigen::MatrixXd> image1, const int width, const int height, const double fov, const double fov1);
Eigen::Matrix<double, 3, Eigen::Dynamic> SfM(Eigen::Ref<const Eigen::MatrixXd> image0, Eigen::Ref<const Eigen::MatrixXd> image1, const int width, const int height, const double fov0, const double fov1, Eigen::Matrix<double, 3, Eigen::Dynamic>& points3d, Eigen::Matrix<double, 3, 4>& pose1);
#endif /* sfm_hpp */
