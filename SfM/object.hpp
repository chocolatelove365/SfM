//
//  object.hpp
//  SfM
//
//  Created by tomiya on 2017/10/06.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#ifndef object_hpp
#define object_hpp

#include <stdio.h>
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include <Eigen/Core>

namespace obj {
    
    void point(GLfloat *vtx);
    void points(GLdouble *vtx, int num);
    void points(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx);
    void lines(GLdouble *vtx, int num);
    void lines(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx);
    
}
#endif /* object_hpp */
