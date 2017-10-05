/**
 This file is part of 8point.
 
 Copyright(C) 2015/2016 Christoph Heindl
 All rights reserved.
 
 This software may be modified and distributed under the terms
 of the BSD license.See the LICENSE file for details.
*/

#include "essential.h"
#include "triangulate.h"
#include "project.h"
#include <Eigen/SVD>
#include <iostream>

namespace eight {
    
    Eigen::Matrix3d essentialMatrix(const Eigen::Matrix3d &k, const Eigen::Matrix3d &f) {
        return k.transpose() * f * k;
    }
    
    Eigen::Matrix<double, 3, 4> pose(const Eigen::Matrix3d &e, const Eigen::Matrix3d &k, Eigen::Ref<const Eigen::MatrixXd> a, Eigen::Ref<const Eigen::MatrixXd> b) {
        
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(e, Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        // Assuming the first camera at identity, there are four possible solutions that need to be tested for
        // the second camera.
        
        Eigen::Matrix3d u = svd.matrixU();
        Eigen::Matrix3d v = svd.matrixV();
        
        if (u.determinant() < 0.0)
            u *= -1.0;
        if (v.determinant() < 0.0)
            v *= -1.0;
        
        
        Eigen::Matrix3d w;
        w <<
            0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 1.0;
        
        
        Eigen::Matrix3d r0 = u * w * v.transpose();
        Eigen::Matrix3d r1 = u * w.transpose() * v.transpose();
        Eigen::Vector3d t = u.col(2);
       
        // Test possible solutions. According to Hartley testing one point for being infront of both cameras should be
        // enough.
        
        Eigen::Matrix<double, 3, 4> camFirst = cameraMatrix(k, Eigen::Matrix<double, 3, 4>::Identity());
        Eigen::Matrix<double, 3, 4> camPosesSecond[4] = {
            cameraPose(r0, t),
            cameraPose(r0, -t),
            cameraPose(r1, t),
            cameraPose(r1, -t)
        };
        
        // Unhandled: triangulate does not work for points at infinity. How to handle these?

        int bestId = 0;
        int bestCount = 0;
        for (int i = 0 ; i < 4; ++i) {

            Eigen::Matrix<double, 3, 4> camSecond = cameraMatrix(k, camPosesSecond[i]);

            Eigen::Matrix<double, 3, Eigen::Dynamic> p = triangulateMany(camFirst, camSecond, a, b);
            Eigen::Matrix<double, 3, Eigen::Dynamic> pSecond = camSecond * p.colwise().homogeneous();
        
            int count = 0;
            for (Eigen::DenseIndex j = 0; j < p.cols(); ++j) {
                if (p(2, j) >= 0.0 && pSecond(2, j) >= 0.0) {
                    ++count;                    
                }
            }
            
            if (count > bestCount) {
                bestCount = count;
                bestId = i;
            }
        }
        
        return camPosesSecond[bestId];
    }
    
}
