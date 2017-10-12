//
//  object.cpp
//  SfM
//
//  Created by tomiya on 2017/10/06.
//  Copyright © 2017年 tomiya. All rights reserved.
//

#include "object.hpp"

namespace obj {
    
    void point(GLdouble *vtx){
        glVertexPointer(3, GL_DOUBLE, 0, vtx);
        glPointSize(4.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        
        glMatrixMode(GL_MODELVIEW);
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_POINTS, 0, 1);
        glDisableClientState(GL_VERTEX_ARRAY);
    }

    void points(GLdouble *vtx, int num){
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
            glutSolidSphere(0.05,16,16);
            glPopMatrix();
        }
    }
    
    void points(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx){
        int num = (int)vtx.cols();
        Eigen::Map<Eigen::RowVectorXd> vec(vtx.data(), vtx.size());
        points((GLdouble*)vec.data(), num);
    }
    
    void lines(GLdouble *vtx, int num){
        glVertexPointer(3, GL_DOUBLE, 0, vtx);
        glLineWidth(2.0f);
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
        
        glEnableClientState(GL_VERTEX_ARRAY);
        glDrawArrays(GL_LINES, 0, num);
        glDisableClientState(GL_VERTEX_ARRAY);
    }
    
    void lines(Eigen::Matrix<double, 3, Eigen::Dynamic> vtx){
        int num = (int)vtx.cols();
        Eigen::Map<Eigen::RowVectorXd> vec(vtx.data(), vtx.size());
        lines((GLdouble*)vec.data(), num);
    }
}
