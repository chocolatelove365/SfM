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

using namespace std;

const int init_width = 960;
const int init_height = 540;
const char *filename = "/Users/tomiya/Desktop/大相撲/video/1709_大相撲9月場所/calib用/calib_image_L.png";
GLuint g_texID;
cv::Mat img;

void setupTexture(GLuint texID, const char *file)
{
    img = cv::imread(file);
    if(img.empty()) cout << "image is empty\n";
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
    float width = img.cols;
    float height = img.rows;
    
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
    gluPerspective(4.375, (double)init_width/init_height, 0.1, 100.0);
}

void draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawTexture();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(4.375, (double)init_width/init_height, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    points();
    lines();
    glutSwapBuffers();
}

void resize(int width, int height){
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(4.375, (double)width/height, 0.1, 100.0);
}

int main(int argc, char * argv[]) {
    const double fov = 4.375 * 2;
    const int width = 1920;
    const int height = 1080;
    const int n_points = 8;
    Eigen::Matrix<double, 2, n_points> image0, image1;
    image0 <<
    1465, 1378, 589, 471, 380, 467, 1215, 1326,
    776, 836, 919, 878, 484, 428, 357, 399;
    image1 <<
    1400, 1288, 518, 434, 553, 659, 1392, 1477,
    772, 817, 764, 707, 328, 289, 337, 392;
    SfM(image0, image1, width, height, fov);
    
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
