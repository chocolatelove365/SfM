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

int WinID[2];

using json = nlohmann::json;
using namespace std;

std::string filename0, filename1;
int width, height;
double fov0, fov1;
Eigen::MatrixXd image0, image1;

const int init_width = 960;
const int init_height = 540;

GLuint g_texID0, g_texID1;
GLuint current_dispID = 0;
cv::Mat img;

Eigen::Vector3d origin;
Eigen::Vector3d x_axis, y_axis, z_axis;
Eigen::Vector3d eye_pos;
Eigen::Matrix<double, 3, Eigen::Dynamic> points3d0, points3d1, points3d_world;
Eigen::Matrix4d camera0;

bool load_info_json(const char *path){
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

bool save_info_json(const char *path){
    std::ofstream output_json(path);
    if(output_json.is_open()){
        json j;
        j["filename0"] = filename0;
        j["filename1"] = filename1;
        j["width"] = width;
        j["height"] = height;
        j["fov0"] = fov0;
        j["fov1"] = fov1;
        j["image0"] = {2, 2};
        j["image1"] = {3, 3};
        output_json << j;
        return true;
    }
    else{
        cout << "ERROR: Could not open json file\n";
        return false;
    }
}

void save_calib_json(const char *path){
    std::ofstream output_json(path);
    json j;
    j["intrinsics"] = {{width, height, fov0, 0.5, 0.5, 0, 0, 0, 0},{width, height, fov1, 0.5, 0.5, 0, 0, 0, 0}};
    j["matrix0"] = {2, 2};
    j["transmat"] = {3, 3};
    output_json << j;
}

void save_model_json(const char *path){
    std::ofstream output_json(path);
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
    
    for(double _fov0 = 5.0; _fov0 < 10.0; _fov0 += 0.1){
        for(double _fov1 = 5.0; _fov1 < 10.0; _fov1 += 0.1){
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

void init_image(){
    glGenTextures(1, &g_texID0);
    setupTexture(g_texID0, filename0.c_str());
    glGenTextures(1, &g_texID1);
    setupTexture(g_texID1, filename1.c_str());
    eye_pos << 50.0, 20.0, 0.0;
}

void disp_image() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if(current_dispID == 0)
        drawTexture(g_texID0);
    else
        drawTexture(g_texID1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(current_dispID == 0)
        gluPerspective(fov0, (double)init_width/init_height, 0.0001, 1000.0);
    else
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
    if(current_dispID == 0){
        draw_lines((Eigen::MatrixXd(3, 6) << origin, origin+x_axis, origin, origin+y_axis, origin, origin+z_axis).finished());
        draw_points(points3d0);
        draw_points2d(image0);
    }
    else{
        draw_points(points3d1);
        draw_points2d(image1);
    }
    glutSwapBuffers();
}

void disp_model(){
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

void idle_image(){
    glutSetWindow(WinID[0]);
    glutPostRedisplay();
}

void reconstruct(){
    Eigen::Matrix<double, 3, 4> tmp;
    Eigen::Matrix4d pose1;
    if(image0.cols() != image1.cols()){
        return;
    }
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
    double scale = 4.55 * 0.5 / x_axis.norm();
    x_axis = x_axis.normalized();
    y_axis = y_axis.normalized();
    z_axis = x_axis.cross(y_axis).normalized();
    y_axis = z_axis.cross(x_axis);
    cout << "x_axis_norm :" << x_axis.norm() << "\n";
    cout << "y_axis_norm :" << y_axis.norm() << "\n";
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
    points3d_world *= scale;
#else
    cout << "x_axis_norm :" << x_axis.norm() << "\n";
    cout << "y_axis_norm :" << y_axis.norm() << "\n";
    z_axis = x_axis.cross(y_axis).normalized() * x_axis.norm();
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
    points3d_world *= 4.55 * 0.5;
#endif
    
    for(int i = 0; i < points3d_world.cols(); i++)
    cout << "points3d_world[" << i <<"]:" << points3d_world.col(i).transpose() << "\n";
}

void key_image(unsigned char key, int x, int y){
    switch(key){
        case '[':
            current_dispID = 0;
            cout << "current_dispID: " << current_dispID << "\n";
            glutSetWindow(WinID[0]);
            glutPostRedisplay();
            break;
        case ']':
            current_dispID = 1;
            cout << "current_dispID: " << current_dispID << "\n";
            glutSetWindow(WinID[0]);
            glutPostRedisplay();
            break;
    }
}

void key_model(unsigned char key, int x, int y){
    time_t currrent_time = std::time(nullptr);
    switch(key){
        case 's':
            cout << "save model json\n";
            save_model_json("model.json");
            break;
        case 'w':
#if 1
            cout << "save info json\n";
            save_info_json("info_1709_day12_east_02.json");
#else
            cout << "save calib json\n";
            save_calib_json("1709_day12_east.json");
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

void special_key_model(int key, int x, int y){
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

void mouse_image(int button, int state, int x, int y){
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
        if(current_dispID == 0)
            add_point(button, state, x, y, image0);
        else
            add_point(button, state, x, y, image1);
        reconstruct();
        glutSetWindow(WinID[0]);
        glutPostRedisplay();
        glutSetWindow(WinID[1]);
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
    const char *path = "/Users/tomiya/Desktop/SfM/SfM/info_1709_day12_east.json";
#else
    const char *path = "/Users/tomiya/Desktop/SfM/SfM/info_1709_day12_west.json";
#endif
    
#if 0
    load_info_json(path);
    calibration();
#else
    load_info_json(path);
#endif
    
    reconstruct();
    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(init_width, init_height);
    glutInitWindowPosition(0, 100);
    WinID[0] = glutCreateWindow("image");
    glutDisplayFunc(disp_image);
    glutIdleFunc(idle_image);
    glutMouseFunc(mouse_image);
    glutKeyboardFunc(key_image);
    init_image();
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(init_width, init_height);
    glutInitWindowPosition(init_width * 2, 100);
    WinID[1] = glutCreateWindow("model");
    glutDisplayFunc(disp_model);
//    glutIdleFunc(idle);
    glutKeyboardFunc(key_model);
    glutSpecialFunc(special_key_model);
    glutMainLoop();
    
    return 0;
}
