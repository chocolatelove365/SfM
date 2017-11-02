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
#include <iostream>
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include <Eigen/Core>

void draw_string(int x, int y, char *string, void *font);
void draw_point(float *vtx);
void draw_points(double *vtx, int num, float size);
void draw_points(Eigen::MatrixXd vtx, float size);
void draw_points(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx, float size);
void draw_lines(double *vtx, int num);
void draw_lines(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx);
void draw_line_loop(double *vtx, int num);
void draw_line_loop2(Eigen::MatrixXd vtx);
void draw_line_loop(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx);
void draw_circle(double x, double y, double z, double radius, int sides);
void draw_coordinate(Eigen::Matrix4d mat);


#endif /* object_hpp */
