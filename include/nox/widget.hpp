 ///////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of nyx, a lightweight C++ template visualization library //
//                                                                            //
// Copyright (C) 2010, 2011 Alexandru Duliu                                   //
//                                                                            //
// nyx is free software; you can redistribute it and/or                       //
// modify it under the terms of the GNU Lesser General Public                 //
// License as published by the Free Software Foundation; either               //
// version 3 of the License, or (at your option) any later version.           //
//                                                                            //
// nyx is distributed in the hope that it will be useful, but WITHOUT ANY     //
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  //
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the //
// GNU General Public License for more details.                               //
//                                                                            //
// You should have received a copy of the GNU Lesser General Public           //
// License along with nyx. If not, see <http://www.gnu.org/licenses/>.        //
//                                                                            //
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <eigen3/Eigen/Core>


namespace nox
{

template <typename T>
class Widget
{
public:
    typedef Eigen::Matrix<T,3,1> Vector3;
    typedef Eigen::Matrix<T,4,4> Matrix4;

public:
    enum MouseButton{ NoButton, LeftButton, RightButton, MiddleButton, X1Button, X2Button };

    Widget()
    {
        // mouse handling
        m_mouseX=0;
        m_mouseY=0;
        m_mouseButton = Widget::NoButton;

        view_transform(0) = static_cast<T>(0.0); // elevation
        view_transform(1) = static_cast<T>(0.0); // azimuth
        view_transform(2) = static_cast<T>(-1.5); // zoom

        m_center = Eigen::Vector3f::Zero();

        // setup the rotation matrix
        m_mv = Eigen::Matrix4f::Identity();
        // rotate 90 deg around x
        m_mv(1,1) = 0;
        m_mv(2,2) = 0;
        m_mv(2,1) =-1;
        m_mv(1,2) = 1;

        m_baseColor(0) = static_cast<T>(0.5);
        m_baseColor(1) = static_cast<T>(0.5);
        m_baseColor(2) = static_cast<T>(0.5);
    }


    virtual ~Widget();

    virtual void initialize()
    {
        // init openGL stuff
        glDisable( GL_DEPTH_TEST );
        glEnable(GL_TEXTURE_2D);
        glEnable(GL_TEXTURE0);
        glEnable( GL_MULTISAMPLE );
        glEnable( GL_BLEND );
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable( GL_POINT_SMOOTH );
        glHint( GL_POINT_SMOOTH_HINT, GL_NICEST );
        glEnable( GL_LINE_SMOOTH );
        glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
        glEnable( GL_POLYGON_SMOOTH );
        glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );

        glLineWidth( 1.0f );
        glPointSize( 5.0f );
    }


    virtual void draw() = 0;


    virtual void handleMousePress( int x, int y, int button )
    {
        m_mouseButton = button;
        m_mouseX = mouseTrans(x);
        m_mouseY = mouseTrans(y);
    }


    virtual void handleMouseMove( int x, int y )
    {
        // new Mouse Movement
        T dx = static_cast<T>( mouseTrans(x) ) - static_cast<T>(m_mouseX);
        T dy = static_cast<T>( mouseTrans(y) ) - static_cast<T>(m_mouseY);

        switch( m_mouseButton )
        {
            case Widget::LeftButton :
                // Update Azimuth & Elevation
                view_transform(0) += dy/static_cast<T>(5.0);
                view_transform(1) += dx/static_cast<T>(5.0);

                // predevent big values
                view_transform[0] = fmod(view_transform(0),static_cast<T>(36000.0));
                view_transform[1] = fmod(view_transform(1),static_cast<T>(36000.0));
                break;

            case Widget::RightButton :
                // Calculate relativ zoom
                float  dist = dy/static_cast<T>(50.0);

                // make zoom logarithmic
                dist *= log((fabs(view_transform(2)/static_cast<T>(10.0))+static_cast<T>(1.0)));

                // update global zoom
                view_transform(2)  += dist;
                break;
        }

        m_mouseX = mouseTrans(x);
        m_mouseY = mouseTrans(y);
    }


    virtual void handleKeyboard()
    {
        // no default behaviour
    }


    virtual void handleResize( int width, int height )
    {
        m_size[0] = width;
        m_size[1] = height;
    }

protected:
    int mouseTrans( int i )
    {
        int result = i;

        // clamp
        int width = m_size[0];
        if( i < 0 || i >width*5 )
            result = 0;
        else if( i > width )
            result = width;

        return result;
    }


    void setModelview( T azimuth, T elevation, T zoom )
    {
        // set modelview
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gl::Translate( 0, 0, zoom);	// Zoom
        gl::Rotate(elevation,1,0,0);	// rotate elevation
        gl::Rotate(azimuth,0,1,0);	// rotate azimuth
        gl::MultMatrix( m_mv.data() );

        //gl::Translate( -m_center(0), -m_center(1), -m_center(2) );
    }

    void setProjection( T fovy, T aspect, T zNear, T zFar )
    {
        // set projection
        glViewport(0,0, m_size[0], m_size[1] );
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        double m[4][4];
        double sine, cotangent, deltaZ;
        double radians = static_cast<double>(fovy) / 2.0 * 3.1415926535897932384626433 / 180.0;

        deltaZ = static_cast<double>(zFar - zNear);
        sine = sin(radians);
        if ((deltaZ == 0) || (sine == 0) || (aspect == 0))
        {
            return;
        }
        cotangent = cos(radians) / sine;

        for( int i=0; i<4; i++ )
            for( int j=0; j<4; j++ )
                m[0][0] = 0.0;

        m[0][0] = static_cast<T>(cotangent / aspect);
        m[1][1] = static_cast<T>(cotangent);
        m[2][2] = static_cast<T>(-(zFar + zNear) / deltaZ);
        m[2][3] = static_cast<T>(-1);
        m[3][2] = static_cast<T>(-2 * zNear * zFar / deltaZ);
        m[3][3] = static_cast<T>(0);
        gl::MultMatrix(&m[0][0]);
    }


    void setView()
    {
        // setProjection
        setProjection( static_cast<T>(60.0),
                       static_cast<T>(m_size[0])/static_cast<T>(m_size[1]),
                       static_cast<T>(0.002),
                       static_cast<T>(1000.0) );

        // set modelview
        setModelview( view_transform(1), view_transform(0), view_transform(2) );
    }

protected:
    // geometry
    int m_size[2];

    // default mouse handling
    int m_mouseX;
    int m_mouseY;
    int m_mouseButton;
    Vector3 view_transform; // elevation, azimutz, zoom
    Vector3 m_center; // center of rotation
    Matrix4 m_mv; // modelview matrix

    // default colors
    Vector3 m_baseColor[3];
};

} // namespace nyx
