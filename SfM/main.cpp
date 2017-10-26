//
//  main.cpp
//  SfM
//
//  Created by tomiya on 2017/09/28.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include <iostream>
#include <sstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GLUT/glut.h>
#include <opencv2/opencv.hpp>
#include "json.hpp"
#include "draw.hpp"
#include "sfm.hpp"

int WinID[3];

using json = nlohmann::json;
using namespace std;

std::string filename0, filename1;
int width, height;
double fov0, fov1;
Eigen::MatrixXd image0, image1;

const int init_width = 960;
const int init_height = 540;

GLuint g_texID0, g_texID1;
cv::Mat img;

Eigen::Vector3d origin;
Eigen::Vector3d x_axis, y_axis, z_axis;
Eigen::Vector3d eye_pos;
Eigen::Matrix<double, 3, Eigen::Dynamic> points3d0, points3d1, points3d_world;
Eigen::Matrix4d camera0;

bool load_json(const char *path){
    std::ifstream input_json(path);
    if(input_json.is_open()){
        json j;
        input_json >> j;
        filename0 = j["filename0"];
        filename1 = j["filename1"];
        width = j["width"];
        height = j["height"];
        fov0 = j["fov0"];
        fov1 = j["fov1"];
        std::vector<double> image0_vec = j["image0"];
        std::vector<double> image1_vec = j["image1"];
        int n_points = 0;
        if(image0_vec.size() == image1_vec.size() && image0_vec.size() % 2 == 0){
            n_points = (int)image0_vec.size() / 2;
            image0 = Eigen::Map<Eigen::MatrixXd>(image0_vec.data(), n_points, 2).transpose();
            image1 = Eigen::Map<Eigen::MatrixXd>(image1_vec.data(), n_points, 2).transpose();
            return true;
        }
        else{
            cout << "ERROR: Image vector sizes are different\n";
            return false;
        }
    }
    else{
        cout << "ERROR: Could not open json file\n";
        return false;
    }
}

void save_json(){
    std::ofstream output_json("calib.json");
    json j;
    j["intrinsics"] = {{width, height, fov0, 0.5, 0.5, 0, 0, 0, 0},{width, height, fov1, 0.5, 0.5, 0, 0, 0, 0}};
    j["matrix0"] = {2, 2};
    j["transmat"] = {3, 3};
    output_json << j;
}

void save_json_model(){
    std::ofstream output_json("model.json");
    json j;
    std::vector<std::vector<double>> model;
    for(int i = 0; i < points3d_world.cols(); i++){
        std::vector<double> point{points3d_world(0, i), points3d_world(1, i), points3d_world(2, i)};
        model.push_back(point);
    }
    j["model"] = model;
    output_json << j;
}

void calibration(){
    double min_diff = DBL_MAX;
    double min_dot = DBL_MAX;
    double best_fov0 = 0.0;
    double best_fov1 = 0.0;
    
    for(double _fov0 = 9.0; _fov0 < 20.0; _fov0 += 0.1){
        for(double _fov1 = 9.0; _fov1 < 20.0; _fov1 += 0.1){
            Eigen::Vector3d _origin, _x_axis, _y_axis;
            Eigen::Matrix<double, 3, Eigen::Dynamic> _points3d;
            Eigen::Matrix<double, 3, 4> pose1;
            _points3d = SfM(image0, image1, width, height, _fov0, _fov1, _points3d, pose1);
            _origin = Eigen::Vector3d::Zero();
            for(int i = 0; i < 8; i++){
                _origin += _points3d.col(i);
            }
            _origin /= 8;
            _x_axis << (_points3d.col(4) + _points3d.col(5)) * 0.5 - _origin;
            _y_axis << (_points3d.col(2) + _points3d.col(3)) * 0.5 - _origin;
            double _diff = abs(_x_axis.norm() - _y_axis.norm());
            double _dot = _x_axis.normalized().dot(_y_axis.normalized());
//            if(min_diff > _diff || min_dot > abs(_dot)){
//            if(min_diff > _diff && abs(_dot) < 0.1){
            if(_diff < 0.01 && min_dot > abs(_dot)){
                min_diff = _diff;
                min_dot = abs(_dot);
                cout << "min_diff: " << min_diff << "\n";
                cout << "min_dot: " << min_dot << "\n";
                best_fov0 = _fov0;
                best_fov1 = _fov1;
                cout << "best_fov0: " << best_fov0 << "\n";
                cout << "best_fov1: " << best_fov1 << "\n";
            }
        }
    }
    fov0 = best_fov0;
    fov1 = best_fov1;
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

void drawTexture(GLuint texID)
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
    glBindTexture(GL_TEXTURE_2D, texID);
    
    // テクスチャの描画
    glEnable(GL_TEXTURE_2D);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);
    glDrawArrays(GL_QUADS, 0, 4);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_TEXTURE_2D);
}

void draw_points2d(Eigen::MatrixXd image){
    int num = (int)image.cols();
    Eigen::Map<Eigen::RowVectorXd> vec(image.data(), image.size());
    GLdouble* vtx = (GLdouble*)vec.data();
    
    glVertexPointer(2, GL_DOUBLE, 0, vtx);
    glPointSize(5.0f);
    glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, height, 0, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, num);
    glDisableClientState(GL_VERTEX_ARRAY);
    for(int i = 0; i < num; i++){
        glPushMatrix();
        glTranslated(vtx[i*2], vtx[i*2+1], 0.0);
        char text[50];
        sprintf(text, "%d", i);
        draw_string(0, 0, text, GLUT_BITMAP_TIMES_ROMAN_24);
        glPopMatrix();
    }
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
}

void init0(){
    glGenTextures(1, &g_texID0);
    setupTexture(g_texID0, filename0.c_str());
    eye_pos << 50.0, 20.0, 0.0;
}

void init1(){
    glGenTextures(1, &g_texID1);
    setupTexture(g_texID1, filename1.c_str());
}

void disp0() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawTexture(g_texID0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov0, (double)init_width/init_height, 0.0001, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const GLdouble base[] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
    glLoadMatrixd(base);
    draw_lines((Eigen::MatrixXd(3, 6) << origin, origin+x_axis, origin, origin+y_axis, origin, origin+z_axis).finished());
    draw_points(points3d0);
    draw_points2d(image0);
    glutSwapBuffers();
}

void disp1() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawTexture(g_texID1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov1, (double)init_width/init_height, 0.0001, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    const GLdouble base[] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, -1.0, 0.0, 0.0,
        0.0, 0.0, -1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
    glLoadMatrixd(base);
    draw_points(points3d1);
    draw_points2d(image1);
    glutSwapBuffers();
}

void disp2(){
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // 3D
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov0, (double)init_width/init_height, 0.0001, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(eye_pos(0), eye_pos(1), eye_pos(2), 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
    draw_points(points3d_world);
    draw_lines((Eigen::MatrixXd(3, 6) <<
                0, 1, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 0, 1).finished());
    draw_lines(points3d_world.leftCols(8));
    draw_line_loop(points3d_world.leftCols(8));
    draw_circle(0.0, 0.0, 0.0, 4.55*0.5, 64);
    draw_circle(0.0, 0.0, 0.0, 1.0, 64);
    draw_line_loop((Eigen::MatrixXd(3, 4) <<
                    2.8, 2.8, -2.8, -2.8,
                    2.8, -2.8, -2.8, 2.8,
                    0.0, 0.0, 0.0, 0.0).finished());
//    draw_coordinate(camera0);
    
    // 2D
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width, 0, height, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    stringstream ss0, ss1;
    ss0 << "fov0: " << fov0;
    draw_string(0, 40, (char*)ss0.str().c_str(), GLUT_BITMAP_TIMES_ROMAN_24);
    ss1 << "fov1: " << fov1;
    draw_string(0, 0, (char*)ss1.str().c_str(), GLUT_BITMAP_TIMES_ROMAN_24);
    
    glutSwapBuffers();
}

void idle0(){
    glutSetWindow(WinID[0]);
    glutPostRedisplay();
}

void reconstruct(){
    Eigen::Matrix<double, 3, 4> tmp;
    Eigen::Matrix4d pose1;
    SfM(image0, image1, width, height, fov0, fov1, points3d0, tmp);
    pose1.block(0, 0, 3, 4) = tmp;
    pose1.row(3) << 0, 0, 0, 1;
    auto points4d1 = pose1.inverse() * points3d0.colwise().homogeneous();
    points3d1 = points4d1.colwise().hnormalized();
    
    for(int i = 0; i < points3d0.cols(); i++) cout << "points3d0[" << i << "]: " << points3d0.col(i).transpose() << "\n";
    for(int i = 0; i < points3d1.cols(); i++) cout << "points3d1[" << i << "]: " << points3d1.col(i).transpose() << "\n";
    
    origin = Eigen::Vector3d::Zero();
    for(int i = 0; i < 8; i++){
        origin += points3d0.col(i);
    }
    origin /= 8;
    x_axis << (points3d0.col(4) + points3d0.col(5)) * 0.5 - origin;
    y_axis << (points3d0.col(2) + points3d0.col(3)) * 0.5 - origin;
    
#if 0
    x_axis = x_axis.normalized();
    y_axis = y_axis.normalized();
    z_axis = x_axis.cross(y_axis).normalized();
    y_axis = z_axis.cross(x_axis);
    Eigen::Matrix4d Rt;
    Rt.block(0, 0, 3, 4) << x_axis, y_axis, z_axis, origin;
    Rt.row(3) << 0, 0, 0, 1;
    cout << "Rt:\n" << Rt << "\n";
    camera0 = Rt.inverse();
    cout << "camera0:\n" << camera0 << "\n";
    auto points4d = points3d0.colwise().homogeneous();
    Eigen::Matrix<double, 4, Eigen::Dynamic> points4d_world = camera0 * points4d;
    points3d_world = Eigen::MatrixXd(3, points4d_world.cols());
    points3d_world << points4d_world.topRows(3);
    points3d_world *= 100.0;
#else
    cout << "x_axis_norm :" << x_axis.norm() << "\n";
    cout << "y_axis_norm :" << y_axis.norm() << "\n";
    double scale = 4.55 * 0.5 / x_axis.norm();
    points3d0.array() *= scale;
    origin.array() *= scale;
//    points3d0 *= scale;
//    origin *= scale;
//    x_axis = x_axis.normalized();
//    y_axis = y_axis.normalized();
    x_axis << (points3d0.col(4) + points3d0.col(5)) * 0.5 - origin;
    y_axis << (points3d0.col(2) + points3d0.col(3)) * 0.5 - origin;
    z_axis = x_axis.cross(y_axis).normalized();
//    y_axis = z_axis.cross(x_axis);
    cout << "z_axis.norm(): " << z_axis.norm() << "\n";
    cout << "dot: " << x_axis.dot(y_axis) << "\n";
    
    Eigen::Matrix4d Rt;
    Rt.block(0, 0, 3, 4) << x_axis, y_axis, z_axis, origin;
    Rt.row(3) << 0, 0, 0, 1;
    cout << "Rt:\n" << Rt << "\n";
    camera0 = Rt.inverse();
    cout << "camera0:\n" << camera0 << "\n";
    
    auto points4d = points3d0.colwise().homogeneous();
    Eigen::Matrix<double, 4, Eigen::Dynamic> points4d_world = camera0 * points4d;
    points3d_world = points4d_world.colwise().hnormalized();

#endif
    for(int i = 0; i < points3d_world.cols(); i++)
    cout << "points3d_world[" << i <<"]:" << points3d_world.col(i).transpose() << "\n";
}

void key(unsigned char key, int x, int y){
    switch(key){
        case 's':
#if 1
            cout << "save model json\n";
            save_json_model();
#else
            cout << "save calib json\n";
            save_json();
#endif
            break;
        case 'c':
            cout << "calibrate\n";
            calibration();
            glutPostRedisplay();
            break;
        case ' ':
            cout << "update\n";
            reconstruct();
            glutPostRedisplay();
            break;
        case 'o':
            fov0 -= 0.1;
            reconstruct();
            glutPostRedisplay();
            break;
        case 'p':
            fov0 += 0.1;
            reconstruct();
            glutPostRedisplay();
            break;
        case '[':
            fov1 -= 0.1;
            reconstruct();
            glutPostRedisplay();
            break;
        case ']':
            fov1 += 0.1;
            reconstruct();
            glutPostRedisplay();
            break;
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
            eye_pos(2) += 10.0;
            glutPostRedisplay();
            break;
        case GLUT_KEY_DOWN:
            eye_pos(2) -= 10.0;
            glutPostRedisplay();
            break;
    }
}

void add_point(int button, int state, int x, int y, Eigen::MatrixXd &image){
    image.conservativeResize(image.rows(), image.cols()+1);
    Eigen::Vector2d pos;
    pos(0) = (double)x / init_width * width;
    pos(1) = (double)y / init_height * height;
    pos(0) = min(max(0, (int)pos(0)), width);
    pos(1) = min(max(0, (int)pos(1)), height);
    image.col(image.cols()-1) = pos;
    cout << "image:\n" << image << "\n";
}

void mouse0(int button, int state, int x, int y){
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
        add_point(button, state, x, y, image0);
        glutPostRedisplay();
    }
}

void mouse1(int button, int state, int x, int y){
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
        add_point(button, state, x, y, image1);
        glutPostRedisplay();
    }
}

//void resize(int width, int height){
//    glViewport(0, 0, width, height);
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    gluPerspective(fov0, (double)width/height, 0.0001, 1000.0);
//}


int main(int argc, char * argv[]) {
#if 1
    const char *path = "/Users/tomiya/Desktop/SfM/SfM/calib_1709_day12_east.json";
#else
    const char *path = "/Users/tomiya/Desktop/SfM/SfM/calib_1709_day12_west.json";
#endif
    
#if 0
    load_json(path);
    calibration();
#else
    load_json(path);
#endif
    
    reconstruct();
    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(init_width, init_height);
    glutInitWindowPosition(0, 100);
    WinID[0] = glutCreateWindow("image0");
    glutDisplayFunc(disp0);
//    glutIdleFunc(idle0);
    glutMouseFunc(mouse0);
    init0();
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(init_width, init_height);
    glutInitWindowPosition(0, 500);
    WinID[1] = glutCreateWindow("image1");
    glutDisplayFunc(disp1);
    glutMouseFunc(mouse1);
    init1();
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(init_width, init_height);
    glutInitWindowPosition(init_width * 2, 100);
    WinID[2] = glutCreateWindow("3D model");
    glutDisplayFunc(disp2);
//    glutIdleFunc(idle);
    glutKeyboardFunc(key);
    glutSpecialFunc(special_key);
    
    glutMainLoop();
    
    return 0;
}
