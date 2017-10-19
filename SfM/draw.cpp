//
//  object.cpp
//  SfM
//
//  Created by tomiya on 2017/10/06.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include "draw.hpp"

void draw_string(int x, int y, char *string, void *font)
{
    int len, i;
    
    glRasterPos2f(x, y);
    len = (int) strlen(string);
    for (i = 0; i < len; i++)
    {
        glutBitmapCharacter(font, string[i]);
    }
    
}

void draw_point(GLdouble *vtx){
    glVertexPointer(3, GL_DOUBLE, 0, vtx);
    glPointSize(4.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    
    glMatrixMode(GL_MODELVIEW);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, 1);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void draw_points(GLdouble *vtx, int num){
    glVertexPointer(3, GL_DOUBLE, 0, vtx);
    glPointSize(4.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    
    glMatrixMode(GL_MODELVIEW);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, num);
    glDisableClientState(GL_VERTEX_ARRAY);
    
    for(int i = 0; i < num; i++){
        glPushMatrix();
        glTranslated(vtx[i*3], vtx[i*3+1], vtx[i*3+2]);
        glutSolidSphere(0.000003,16,16);
        char text[50];
        sprintf(text, "%d", i);
        draw_string(0, 0, text, GLUT_BITMAP_TIMES_ROMAN_24);
        glPopMatrix();
    }
}

void draw_points(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx){
    int num = (int)vtx.cols();
    Eigen::Map<Eigen::RowVectorXd> vec(vtx.data(), vtx.size());
    draw_points((GLdouble*)vec.data(), num);
}

void draw_lines(GLdouble *vtx, int num){
    glVertexPointer(3, GL_DOUBLE, 0, vtx);
    glLineWidth(2.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_LINES, 0, num);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void draw_lines(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx){
    int num = (int)vtx.cols();
    Eigen::Map<Eigen::RowVectorXd> vec(vtx.data(), vtx.size());
    draw_lines((GLdouble*)vec.data(), num);
}

void draw_line_loop(GLdouble *vtx, int num){
    glVertexPointer(3, GL_DOUBLE, 0, vtx);
    glLineWidth(2.0f);
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_LINE_LOOP, 0, num);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void draw_line_loop(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx){
    int num = (int)vtx.cols();
    Eigen::Map<Eigen::RowVectorXd> vec(vtx.data(), vtx.size());
    draw_line_loop((GLdouble*)vec.data(), num);
}

void draw_circle(double x, double y, double z, double radius, int sides){
    Eigen::MatrixXd vtx(3, sides);
    for(int i = 0; i < sides; i++){
        double x = cos(i * 2 * M_PI / sides) * radius;
        double y = sin(i * 2 * M_PI / sides) * radius;
        vtx.col(i) = Eigen::Vector3d(x, y, z);
    }
    draw_line_loop(vtx);
}

void draw_coordinate(Eigen::Matrix4d mat){
    Eigen::Vector3d x_axis = mat.block(0, 0, 3, 1);
    Eigen::Vector3d y_axis = mat.block(0, 1, 3, 1);
    Eigen::Vector3d z_axis = mat.block(0, 2, 3, 1);
    Eigen::Vector3d origin = mat.block(0, 3, 3, 1);
    draw_lines((Eigen::MatrixXd(3, 6) << origin, origin+x_axis, origin, origin+y_axis, origin, origin+z_axis).finished());
}

