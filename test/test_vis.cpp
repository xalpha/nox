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

#include <GL/glew.h>
#include <GL/glut.h>

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

    // Enter GLUT main loop. Main loop will be terminated only
    // on user request by exit(0). Other means do not currently exist in GLUT.
    glutMainLoop();

    return 0;
}

