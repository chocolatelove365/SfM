//
//  main.cpp
//  SfM
//
//  Created by tomiya on 2017/09/28.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include <iostream>
#include <Eigen/Core>
#include <GLUT/glut.h>
#include <opencv2/opencv.hpp>
#include "object.hpp"
#include "sfm.hpp"

#define POINTS_8 0

using namespace std;

const double fov = 4.375;
//const double fov = 6.0;
const int width = 1920;
const int height = 1080;
#if POINTS_8
const int n_points = 8;
#else
const int n_points = 22;
#endif

const int init_width = 960;
const int init_height = 540;
const char *filename = "/Users/tomiya/Desktop/大相撲/video/1709_大相撲9月場所/calib用/calib_image_L.png";
GLuint g_texID;
cv::Mat img;

Eigen::Vector3d origin;
Eigen::Matrix<double, 3, 8> xyz_axis;
Eigen::Matrix<double, 3, Eigen::Dynamic> points3d;

void setupTexture(GLuint texID, const char *file)
{
    img = cv::imread(file);
    if(img.empty()){
        cout << "image is empty\n";
        return;
    }
    if(img.cols != width || img.rows != height){
        cout << "the size of image is incorrect\n";
        return;
    }
    cv::cvtColor(img, img, CV_BGR2RGB);
    
    glBindTexture(GL_TEXTURE_2D, texID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, img.data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}

void drawTexture()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, 0, height, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    static const GLfloat vtx[] = {
        0.0, 0.0, 0.0,
        width, 0.0, 0.0,
        width, height, 0.0,
        0.0, height, 0.0
    };
    glVertexPointer(3, GL_FLOAT, 0, vtx);
    
    // テクスチャの領域指定
    static const GLfloat texuv[] = {
        0.0f, 1.0f,
        1.0f, 1.0f,
        1.0f, 0.0f,
        0.0f, 0.0f,
    };
    glTexCoordPointer(2, GL_FLOAT, 0, texuv);
    
    // テクスチャの画像指定
    glBindTexture(GL_TEXTURE_2D, g_texID);
    
    // テクスチャの描画
    glEnable(GL_TEXTURE_2D);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glDrawArrays(GL_QUADS, 0, 4);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_TEXTURE_2D);
}

void init(){
    glGenTextures(1, &g_texID);
    setupTexture(g_texID, filename);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glViewport(0, 0, init_width, init_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (double)init_width/init_height, 0.0001, 1000.0);
}

void draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawTexture();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (double)init_width/init_height, 0.0001, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    obj::lines(xyz_axis);
    obj::points(points3d);
    glutSwapBuffers();
}

void resize(int width, int height){
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (double)width/height, 0.0001, 1000.0);
}

int main(int argc, char * argv[]) {
    Eigen::Matrix<double, 2, n_points> image0, image1;
#if POINTS_8
    image0 <<
    1465, 1378, 589, 471, 380, 467, 1215, 1326,
    776, 836, 919, 878, 484, 428, 357, 399;
    image1 <<
    1400, 1288, 518, 434, 553, 659, 1392, 1477,
    772, 817, 764, 707, 328, 289, 337, 392;
#else
    image0 <<
    1465, 1378, 589, 471, 380, 467, 1215, 1326, 929, 756, 897, 625, 985, 1190, 1127, 787, 512, 132, 1764, 1712,  773, 150,
     776,  836, 919, 878, 484, 428,  357, 399,  712, 652, 556, 459, 468,  512,  191, 276, 472, 404,  504,  609, 1002, 757;
    image1 <<
    1400, 1288, 518, 434, 553, 659, 1392, 1477, 940, 812, 990, 613, 1019, 1198, 1465, 1038, 633, 394, 1843, 1734, 645, 213,
     772,  817, 764, 707, 328, 289,  337,  392, 627, 542, 474, 329,  400,  475,  169,  194, 332, 218,  563,  655, 872, 544;
#endif
    points3d = SfM(image0, image1, width, height, fov);
    cout << "points3d:\n" << points3d << "\n";
    origin = Eigen::Vector3d::Zero();
    for(int i = 0; i < 8; i++){
        origin += points3d.col(i);
    }
    origin /= 8;
    
    cout << "origin:\n" << origin << "\n";
    
    xyz_axis <<
    origin(0), (points3d(0, 0) + points3d(0, 1)) * 0.5, origin(0), (points3d(0, 2) + points3d(0, 3)) * 0.5, origin(0), (points3d(0, 4) + points3d(0, 5)) * 0.5, origin(0), (points3d(0, 6) + points3d(0, 7)) * 0.5,
    origin(1), (points3d(1, 0) + points3d(1, 1)) * 0.5, origin(1), (points3d(1, 2) + points3d(1, 3)) * 0.5, origin(1), (points3d(1, 4) + points3d(1, 5)) * 0.5, origin(1), (points3d(1, 6) + points3d(1, 7)) * 0.5,
    origin(2), (points3d(2, 0) + points3d(2, 1)) * 0.5, origin(2), (points3d(2, 2) + points3d(2, 3)) * 0.5, origin(2), (points3d(2, 4) + points3d(2, 5)) * 0.5, origin(2), (points3d(2, 6) + points3d(2, 7)) * 0.5;
    
    glutInit(&argc, argv);
    glutInitWindowSize(init_width, init_height);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("SfM");
    glutReshapeFunc(resize);
    glutDisplayFunc(draw);
    init();
    glutMainLoop();
    
    return 0;
}
