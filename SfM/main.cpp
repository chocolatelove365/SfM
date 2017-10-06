//
//  main.cpp
//  SfM
//
//  Created by tomiya on 2017/09/28.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include <iostream>
#include <GLUT/glut.h>
#include "object.hpp"
#include "sfm.hpp"

using namespace std;

const int init_width = 960;
const int init_height = 540;

void init(){
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glViewport(0, 0, init_width, init_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(10.0, (double)init_width/init_height, 0.1, 100.0);
}

void draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
//    glutSolidTeapot(0.5);
    points();
    lines();
    glutSwapBuffers();
}

void resize(int width, int height){
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(10.0, (double)width/height, 0.1, 100.0);
}

int main(int argc, char * argv[]) {
    glutInit(&argc, argv);
    glutInitWindowSize(init_width, init_height);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("SfM");
    glutReshapeFunc(resize);
    glutDisplayFunc(draw);
    init();
    glutMainLoop();
//    SfM();
    return 0;
}
