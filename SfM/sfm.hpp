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

Eigen::Matrix<double, 3, Eigen::Dynamic> SfM(Eigen::Ref<const Eigen::MatrixXd> image0, Eigen::Ref<const Eigen::MatrixXd> image1, const int width, const int height, const double fov);

#endif /* sfm_hpp */
