//
//  main.cpp
//  SfM
//
//  Created by tomiya on 2017/09/28.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GLUT/glut.h>
#include <opencv2/opencv.hpp>
#include "json.hpp"
#include "object.hpp"
#include "sfm.hpp"

using json = nlohmann::json;
using namespace std;

std::string filename;
int width, height;
double fov;
int n_points;
Eigen::MatrixXd image0, image1;

const int init_width = 960;
const int init_height = 540;

GLuint g_texID;
cv::Mat img;

Eigen::Vector3d origin;
Eigen::Vector3d x_axis, y_axis, z_axis;
Eigen::Vector3d eye_pos;
Eigen::Matrix<double, 3, Eigen::Dynamic> points3d, points3d_world;


void load(){
    std::ifstream input_json("/Users/tomiya/Desktop/SfM/SfM/image_data.json");
    if(input_json.is_open()){
        json json_obj;
        input_json >> json_obj;
        filename = json_obj["filename"];
        width = json_obj["width"];
        height = json_obj["height"];
        fov = json_obj["fov"];
        n_points = json_obj["n_points"];
        std::vector<double> image0_vec = json_obj["image0"];
        image0 = Eigen::Map<Eigen::MatrixXd>(image0_vec.data(), n_points, 2).transpose();
        std::vector<double> image1_vec = json_obj["image1"];
        image1 = Eigen::Map<Eigen::MatrixXd>(image1_vec.data(), n_points, 2).transpose();
    }
    else cout << "Could not open json file\n";
}

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
        (GLfloat)width, 0.0, 0.0,
        (GLfloat)width, (GLfloat)height, 0.0,
        0.0, (GLfloat)height, 0.0
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
    setupTexture(g_texID, filename.c_str());
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glViewport(0, 0, init_width, init_height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (double)init_width/init_height, 0.0001, 1000.0);
    eye_pos << 50.0, 20.0, 0.0;
}

void draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawTexture();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (double)init_width/init_height, 0.0001, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    obj::lines((Eigen::MatrixXd(3, 6) << origin, origin+x_axis, origin, origin+y_axis, origin, origin+z_axis).finished());
    obj::points(points3d);
    glutSwapBuffers();
}

void draw_model(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (double)init_width/init_height, 0.0001, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye_pos(0), eye_pos(1), eye_pos(2), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    obj::points(points3d_world);
    obj::lines((Eigen::MatrixXd(3, 6) <<
                0, 1, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 1).finished());
    obj::lines(points3d_world.leftCols(8));
    obj::line_loop(points3d_world.leftCols(8));
    glutSwapBuffers();
}

void key(unsigned char key, int x, int y){
    switch(key){
    case 'a':
        cout << "a key\n";
    }
}

void special_key(int key, int x, int y){
    switch(key){
        case GLUT_KEY_LEFT:
            eye_pos(0) += 2.0;
            glutPostRedisplay();
            break;
        case GLUT_KEY_RIGHT:
            eye_pos(0) -= 2.0;
            glutPostRedisplay();
            break;
        case GLUT_KEY_UP:
            eye_pos(2) += 2.0;
            glutPostRedisplay();
            break;
        case GLUT_KEY_DOWN:
            eye_pos(2) -= 2.0;
            glutPostRedisplay();
            break;
    }
}

void resize(int width, int height){
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, (double)width/height, 0.0001, 1000.0);
}

int main(int argc, char * argv[]) {
    load();
    
    points3d = SfM(image0, image1, width, height, fov);
    origin = Eigen::Vector3d::Zero();
    for(int i = 0; i < 8; i++){
        origin += points3d.col(i);
    }
    origin /= 8;
    
    cout << "origin:\n" << origin << "\n";
    x_axis << ((points3d.col(4) + points3d.col(5)) * 0.5 - origin).normalized();
    y_axis << ((points3d.col(2) + points3d.col(3)) * 0.5 - origin).normalized();
    z_axis = x_axis.cross(y_axis);
    cout << "dot: " << x_axis.dot(y_axis) << "\n";
    Eigen::Matrix4d Rt;
    Rt.block(0, 0, 3, 4) << x_axis, y_axis, z_axis, origin;
    Rt.row(3) << 0, 0, 0, 1;
    cout << "Rt:\n" << Rt << "\n";

    Eigen::MatrixXd points4d(4, points3d.cols());
    points4d.block(0, 0, 3, points3d.cols()) << points3d;
    points4d.row(3) = Eigen::RowVectorXd::Ones(points3d.cols());
    Eigen::Matrix<double, 4, Eigen::Dynamic> points4d_world = Rt.inverse() * points4d;
    points3d_world = Eigen::MatrixXd(3, points4d_world.cols());
    points3d_world << points4d_world.topRows(3);
    cout << "points3d_world:\n" << points3d_world << "\n";
    
    Eigen::Vector3d pos = points3d.col(11) - origin;
    cout << pos.dot(x_axis) << ", " << pos.dot(y_axis) << ", " << pos.dot(z_axis) << "\n";
    
    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(init_width, init_height);
    glutInitWindowPosition(0, 100);
    glutCreateWindow("SfM");
    glutReshapeFunc(resize);
    glutDisplayFunc(draw);
    init();
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(init_width, init_height);
    glutInitWindowPosition(init_width * 2, 100);
    glutCreateWindow("3D model");
    glutDisplayFunc(draw_model);
    glutKeyboardFunc(key);
    glutSpecialFunc(special_key);
    
    glutMainLoop();
    
    return 0;
}
