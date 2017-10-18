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
double fov0, fov1;
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
Eigen::Matrix4d camera0;

void load_json(const char *path){
    std::ifstream input_json(path);
    if(input_json.is_open()){
        json j;
        input_json >> j;
        filename = j["filename"];
        width = j["width"];
        height = j["height"];
        fov0 = j["fov0"];
        fov1 = j["fov1"];
        n_points = j["n_points"];
        std::vector<double> image0_vec = j["image0"];
        image0 = Eigen::Map<Eigen::MatrixXd>(image0_vec.data(), n_points, 2).transpose();
        std::vector<double> image1_vec = j["image1"];
        image1 = Eigen::Map<Eigen::MatrixXd>(image1_vec.data(), n_points, 2).transpose();
    }
    else cout << "Could not open json file\n";
}

void save_json(){
    std::ofstream output_json("calib.json");
    json j;
    j["intrinsics"] = {{width, height, fov0, 0.5, 0.5, 0, 0, 0, 0},{width, height, fov1, 0.5, 0.5, 0, 0, 0, 0}};
    j["matrix0"] = {2, 2};
    j["transmat"] = {3, 3};
    output_json << j;
}

void calibration(){
    double min_diff = DBL_MAX;
    double min_dot = DBL_MAX;
    double best_fov0 = 0.0;
    double best_fov1 = 0.0;
    
    for(double _fov0 = 2.0; _fov0 < 10.0; _fov0 += 0.1){
        for(double _fov1 = 2.0; _fov1 < 10.0; _fov1 += 0.1){
            Eigen::Vector3d _origin, _x_axis, _y_axis;
            Eigen::Matrix<double, 3, Eigen::Dynamic> _points3d;
            _points3d = SfM(image0, image1, width, height, _fov0, _fov1);
            _origin = Eigen::Vector3d::Zero();
            for(int i = 0; i < 8; i++){
                _origin += _points3d.col(i);
            }
            _origin /= 8;
            _x_axis << (_points3d.col(4) + _points3d.col(5)) * 0.5 - _origin;
            _y_axis << (_points3d.col(2) + _points3d.col(3)) * 0.5 - _origin;
            double _diff = abs(_x_axis.norm() - _y_axis.norm());
            double _dot = _x_axis.normalized().dot(_y_axis.normalized());
//            if(min_diff > _diff && min_dot > abs(_dot)){
            if(min_diff > _diff && abs(_dot) < 0.1){
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
    gluPerspective(fov0, (double)init_width/init_height, 0.0001, 1000.0);
    eye_pos << 50.0, 20.0, 0.0;
}

void draw() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawTexture();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov0, (double)init_width/init_height, 0.0001, 1000.0);
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
    gluPerspective(fov0, (double)init_width/init_height, 0.0001, 1000.0);
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
    obj::circle(0.0, 0.0, 0.0, 4.55*0.5, 64);
    obj::line_loop((Eigen::MatrixXd(3, 4) <<
                    2.8, 2.8, -2.8, -2.8,
                    2.8, -2.8, -2.8, 2.8,
                    0.0, 0.0, 0.0, 0.0).finished());
    obj::coordinate(camera0);
    glutSwapBuffers();
}

void key(unsigned char key, int x, int y){
    switch(key){
        case 's':
            save_json();
            cout << "save calib json\n";
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

void mouse(int button, int state, int x, int y){
    if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
        cout << "x: " << x << ", y: " << y << "\n";
        image0.conservativeResize(image0.rows(), image0.cols()+1);
        Eigen::Vector2d pos;
        pos << (double)x/init_width*width, (double)y/init_height*height;
        image0.col(image0.cols()-1) = pos;
        cout << "image0:\n" << image0 << "\n";
    }
    glutPostRedisplay();
}

void resize(int width, int height){
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov0, (double)width/height, 0.0001, 1000.0);
}

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
    points3d = SfM(image0, image1, width, height, fov0, fov1);
    origin = Eigen::Vector3d::Zero();
    for(int i = 0; i < 8; i++){
        origin += points3d.col(i);
    }
    origin /= 8;
    x_axis << (points3d.col(4) + points3d.col(5)) * 0.5 - origin;
    y_axis << (points3d.col(2) + points3d.col(3)) * 0.5 - origin;
    
    cout << "x_axis_norm :" << x_axis.norm() << "\n";
    cout << "y_axis_norm :" << y_axis.norm() << "\n";
    double scale = 4.55 * 0.5 / y_axis.norm();
    points3d.array() *= scale;
    origin.array() *= scale;
    x_axis = x_axis.normalized();
    y_axis = y_axis.normalized();
    z_axis = x_axis.cross(y_axis);
    cout << "dot: " << x_axis.dot(y_axis) << "\n";
    
    Eigen::Matrix4d Rt;
    Rt.block(0, 0, 3, 4) << x_axis, y_axis, z_axis, origin;
    Rt.row(3) << 0, 0, 0, 1;
    cout << "Rt:\n" << Rt << "\n";
    camera0 = Rt.inverse();
    cout << "camera0:\n" << camera0 << "\n";
    
    Eigen::MatrixXd points4d(4, points3d.cols());
    points4d.block(0, 0, 3, points3d.cols()) << points3d;
    points4d.row(3) = Eigen::RowVectorXd::Ones(points3d.cols());
    Eigen::Matrix<double, 4, Eigen::Dynamic> points4d_world = camera0 * points4d;
    points3d_world = Eigen::MatrixXd(3, points4d_world.cols());
    points3d_world << points4d_world.topRows(3);
    
    for(int i = 0; i < points4d_world.cols(); i++)
        cout << "points3d_world[" << i <<"]:" << points3d_world.col(i).transpose() << "\n";
    
    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(init_width, init_height);
    glutInitWindowPosition(0, 100);
    glutCreateWindow("SfM");
    glutReshapeFunc(resize);
    glutDisplayFunc(draw);
    glutMouseFunc(mouse);
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
