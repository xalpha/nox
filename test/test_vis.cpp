///////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of nox, a lightweight C++ template library for OpenGL    //
//                                                                            //
// Copyright (C) 2012 Alexandru Duliu                                         //
//                                                                            //
// nox is free software; you can redistribute it and/or                       //
// modify it under the terms of the GNU Lesser General Public                 //
// License as published by the Free Software Foundation; either               //
// version 3 of the License, or (at your option) any later version.           //
//                                                                            //
// nox is distributed in the hope that it will be useful, but WITHOUT ANY     //
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  //
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the //
// GNU General Public License for more details.                               //
//                                                                            //
// You should have received a copy of the GNU Lesser General Public           //
// License along with nox. If not, see <http://www.gnu.org/licenses/>.        //
//                                                                            //
///////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdint.h>

#include <GL/glew.h>
#include <GL/glut.h>

#include <Eigen/Geometry>

#include <nox/vis.hpp>

/*
 * test_vis.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: alex
 */


/////
// Instances
///
static nox::vis<float> s_vis;
static size_t count = 10000;


/////
// GLUT callbacks
///
void idlefunc(void){ glutPostRedisplay(); }
void reshapefunc(int width,int height){ s_vis.handleResize( width, height ); }
void motionfunc(int x,int y){ s_vis.handleMouseMove( x,y ); }
void keyboardfunc(unsigned char key,int x,int y){ s_vis.handleKeyboard( key ); }
void displayfunc(){ s_vis.draw(); glutSwapBuffers(); }
void mousefunc(int button,int state,int x,int y)
{
    switch( button )
    {
        case GLUT_LEFT_BUTTON : s_vis.handleMousePress( x,y,nox::vis<float>::LeftButton ); break;
        case GLUT_RIGHT_BUTTON : s_vis.handleMousePress( x,y,nox::vis<float>::RightButton ); break;
    }
}


/////
// Test Plotting functionds
//
void fuzzyCube( const Eigen::Vector3f& pos, float size, uint32_t flags )
{
    std::vector<Eigen::Vector3f> points;
    points.reserve(count);
    for( size_t i=0; i<count; i++ )
        points.push_back( size * Eigen::Vector3f::Random() + pos );

    s_vis.plot( points, flags );
}


void fuzzyAffines()
{
    std::vector<Eigen::Matrix4f> trans;
    trans.reserve(count/10);
    for( size_t i=0; i<count/10; i++ )
    {
        Eigen::Vector3f x = Eigen::Vector3f::Random();
        Eigen::Vector3f y = Eigen::Vector3f::Random();

        x.normalize();
        y.normalize();

        Eigen::Vector3f z = x.cross(y);
        z.normalize();

        y = z.cross(x);
        y.normalize();

        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        Eigen::Matrix3f r = Eigen::Matrix3f::Identity();

        r.col(0) = x;
        r.col(1) = y;
        r.col(2) = z;

        t.rotate(r);
        t.translate( 0.5f * Eigen::Vector3f::Random() + Eigen::Vector3f(0.5,0.5,0.5) );

        trans.push_back( t.matrix() );
    }

    s_vis.plot( trans, nox::vis<float>::Black | nox::vis<float>::CS );
}


void alignedAffines()
{
    std::vector<Eigen::Matrix4f> trans;
    trans.reserve(count);
    for( size_t i=0; i<count; i++ )
    {
        Eigen::Affine3f t = Eigen::Affine3f::Identity();
        t.rotate(Eigen::Matrix3f::Identity());
        t.translate( 0.5f * Eigen::Vector3f::Random() + Eigen::Vector3f(0.5,0.5,0.5) );

        trans.push_back( t.matrix() );
    }

    s_vis.plot( trans, nox::vis<float>::Black | nox::vis<float>::CS );
}



int main( int argc, char** argv )
{
    // init glut
    glutInit(&argc,argv);

    // initial size of viewport (client area of window)
    glutInitWindowSize(800,600);

    // Requests a double RGB-framebuffer with depth- and stencil-buffer
    glutInitDisplayMode( GLUT_DOUBLE| GLUT_RGBA| GLUT_DEPTH );

    // Creates the window.
    glutCreateWindow( "nox: test vis" );

    // init GLEW
    int err = glewInit();
    if (GLEW_OK != err)	{
        // problem: glewInit failed, something is seriously wrong
        std::cerr << "nox::test_widget: GLEW error: \"" << glewGetErrorString(err) << "\"." << std::endl;

        exit(-1);
    }

    // Register the callback-functions. These are then executed by GLUT
    // as soon as the main loop is entered (see below)
    glutDisplayFunc(displayfunc);
    glutReshapeFunc(reshapefunc);
    glutMouseFunc(mousefunc);
    glutMotionFunc(motionfunc);
    glutKeyboardFunc(keyboardfunc);
    glutIdleFunc(idlefunc);

    // plot some stuff
    s_vis.initialize();
    fuzzyCube( Eigen::Vector3f(0,0,0), 0.25, nox::vis<float>::Black );
    fuzzyCube( Eigen::Vector3f(1,0,0), 0.25, nox::vis<float>::Red );
    fuzzyCube( Eigen::Vector3f(0,1,0), 0.25, nox::vis<float>::Green );
    fuzzyCube( Eigen::Vector3f(0,0,1), 0.25, nox::vis<float>::Blue );
    fuzzyCube( Eigen::Vector3f(1,1,0), 0.25, nox::vis<float>::Yellow );
    fuzzyCube( Eigen::Vector3f(0,1,1), 0.25, nox::vis<float>::Cyan );
    fuzzyCube( Eigen::Vector3f(1,0,1), 0.25, nox::vis<float>::Magenta );
    fuzzyCube( Eigen::Vector3f(1,1,1), 0.25, nox::vis<float>::Orange );
    fuzzyAffines();
    alignedAffines();

    // Enter GLUT main loop. Main loop will be terminated only
    // on user request by exit(0). Other means do not currently exist in GLUT.
    glutMainLoop();

    return 0;
}

