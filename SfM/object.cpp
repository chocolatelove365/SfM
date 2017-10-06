//
//  object.cpp
//  SfM
//
//  Created by tomiya on 2017/10/06.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include "object.hpp"
#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>

GLdouble vertex[][3] = {
    { 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0 },
    { 1.0, 1.0, 0.0 },
    { 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 1.0 },
    { 1.0, 0.0, 1.0 },
    { 1.0, 1.0, 1.0 },
    { 0.0, 1.0, 1.0 }
};

int edge[][2] = {
    { 0, 1 },
    { 1, 2 },
    { 2, 3 },
    { 3, 0 },
    { 4, 5 },
    { 5, 6 },
    { 6, 7 },
    { 7, 4 },
    { 0, 4 },
    { 1, 5 },
    { 2, 6 },
    { 3, 7 }
};

void sample(){
    glColor3d(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    for (int i = 0; i < 12; ++i) {
        glVertex3dv(vertex[edge[i][0]]);
        glVertex3dv(vertex[edge[i][1]]);
    }
}

void points(){
    static const GLfloat vtx1[] = {
        -1.6603, -0.776775, -40.9718,
        -1.35433, -0.959523, -40.3693,
        0.924411, -0.946867, -31.1242,
        1.15697, -0.801709, -29.5447,
        1.23062, 0.117859, -26.4906,
        1.0744, 0.243241, -27.2134,
        -0.731858, 0.522756, -35.732,
        -1.09251, 0.41882, -37.1854
    };
    glVertexPointer(3, GL_FLOAT, 0, vtx1);
    glPointSize(4.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    
    glMatrixMode(GL_MODELVIEW);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, 8);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void lines(){
    static const GLfloat vtx2[] = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
    };
    glVertexPointer(3, GL_FLOAT, 0, vtx2);
    glLineWidth(4.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_LINES, 0, 2);
    glDisableClientState(GL_VERTEX_ARRAY);
}
