 ///////////////////////////////////////////////////////////////////////////////
//                                                                            //
// This file is part of nox, a lightweight C++ template visualization library //
//                                                                            //
// Copyright (C) 2010, 2011 Alexandru Duliu                                   //
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

#pragma once

#include <list>
#include <numeric>

#include <Eigen/Core>

#include <nyx/vertex_buffer_object.hpp>

#include <nox/widget.hpp>

/*
 *  vis.hpp
 *
 *  Created on: Aug 16, 2010
 *      Author: alex
 */


namespace nox
{

template <typename T>
class vis : public widget<T>
{
public:
    typedef Eigen::Matrix<T,2,1> Vector2;
    typedef Eigen::Matrix<T,3,1> Vector3;
    typedef Eigen::Matrix<T,4,1> Vector4;
    typedef Eigen::Matrix<T,4,4> Matrix4;
    typedef std::list< nyx::vertex_buffer_object<float,unsigned int> > BufferList;

    // The flags define:
    // Colors:
    // What to draw
    // How to draw it
    enum Flags
    {
        Red     = 0x00000001,
        Green   = 0x00000002,
        Blue    = 0x00000004,
        Orange  = 0x00000008,
        Black   = 0x00000010,
        Gray    = 0x00000020,
        Alpha1  = 0x00001000,
        Alpha075= 0x00002000,
        Alpha05 = 0x00004000,
        Alpha025= 0x00008000,
        Pos     = 0x01000000,
        CS      = 0x01000000
    };

public:
    vis() : widget<T>()
    {
        // init coordinate system geometry
        m_cs_vertices.push_back( Vector3(0,0,0) );
        m_cs_vertices.push_back( Vector3(1,0,0) );
        m_cs_vertices.push_back( Vector3(0,0,0) );
        m_cs_vertices.push_back( Vector3(0,1,0) );
        m_cs_vertices.push_back( Vector3(0,0,0) );
        m_cs_vertices.push_back( Vector3(0,0,1) );

        m_cs_colors.push_back( Vector4(1,0,0,1) );
        m_cs_colors.push_back( Vector4(1,0,0,1) );
        m_cs_colors.push_back( Vector4(0,1,0,1) );
        m_cs_colors.push_back( Vector4(0,1,0,1) );
        m_cs_colors.push_back( Vector4(0,0,1,1) );
        m_cs_colors.push_back( Vector4(0,0,1,1) );

        // init min
        m_min(0) = std::numeric_limits<T>::max();
        m_min(1) = std::numeric_limits<T>::max();
        m_min(2) = std::numeric_limits<T>::max();

        // init max
        m_max(0) = std::numeric_limits<T>::min();
        m_max(0) = std::numeric_limits<T>::min();
        m_max(0) = std::numeric_limits<T>::min();
    }


    virtual ~vis(){}


    void draw()
    {
        // clear buffer
        nyx::gl::ClearColor( nox::widget<T>::m_baseColor );
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // set the mvp matrix
        widget<T>::setView();

        // draw one big CS smack in the origin
        glLineWidth( 2.0f );
        drawCS(1.0f);
        glLineWidth( 1.5f );

        // draw all transformations
        for( BufferList::iterator it=m_buffers.begin(); it!=m_buffers.end(); it++ )
            it->draw();
    }


    template <typename R>
    void plot( const Eigen::Matrix<R,4,4> &trans, uint32_t flags )
    {
        std::vector<Eigen::Matrix<R,4,4> > vec;
        vec.push_back( trans );
        plot( vec, flags );
    }


    template <typename R>
    void plot( const std::vector<Eigen::Matrix<R,4,4> > &trans, uint32_t flags )
    {
        // init stuff
        std::vector<T> pointsV;
        std::vector<T> pointsC;
        std::vector<T> linesV;
        std::vector<T> linesC;

        // color
        Vector4 col = color( flags );

        for( size_t i=0; i<trans.size(); i++ )
        {
            float fac = 1.0f - 0.75f*( float(i)/float(trans.size()) );

            pointsV.push_back( static_cast<T>( trans[i](12) ) );
            pointsV.push_back( static_cast<T>( trans[i](13) ) );
            pointsV.push_back( static_cast<T>( trans[i](14) ) );

            pointsC.push_back( col(0) );
            pointsC.push_back( col(1) );
            pointsC.push_back( col(2) );
            pointsC.push_back( col(3) * fac );

            for( size_t j=0; bool(flags & CS) & (j<m_cs_vertices.size()); j++ )
            {
                Vector4 v( 0.1f*m_cs_vertices[j](0),
                           0.1f*m_cs_vertices[j](1),
                           0.1f*m_cs_vertices[j](2), 1 );
                v = trans[i].template cast<T>() * v;

                for( size_t k=0; k<3; k++ )
                    linesV.push_back( v(k) / v(3) );

                for( size_t k=0; k<4; k++ )
                    linesC.push_back( m_cs_colors[j](k) * ((k==3) ? fac : 1.0f) );
            }
        }

        addGeometry( pointsV, pointsC, GL_POINTS );
        addGeometry( linesV, linesC, GL_LINES );
    }


    void clear()
    {
        m_buffers.clear();
    }


    void drawCS( float s )
    {
        glBegin( GL_LINES );
            for( size_t i=0; i<m_cs_vertices.size(); i++ )
            {
                nyx::gl::Color4( m_cs_colors[i].data() );
                nyx::gl::Vertex3( m_cs_vertices[i].data() );
            }
        glEnd();
    }


    Vector4 color( uint32_t flags )
    {
        // get the alpha
        T a = 1;
        if( flags & Alpha1 ) a = 1;
        else if( flags & Alpha075 ) a = static_cast<T>(0.75);
        else if( flags & Alpha05 ) a = static_cast<T>(0.5);
        else if( flags & Alpha025 ) a = static_cast<T>(0.25);

        // get the color
        Vector4 col(0,0,0,a);
        if( flags & Red )         col = Vector4f( 1,0,0,a );
        else if( flags & Green )  col = Vector4f( 0,1,0,a );
        else if( flags & Blue )   col = Vector4f( 0,0,1,a );
        else if( flags & Orange ) col = Vector4f( 1,0.5,0,a );
        else if( flags & Black )  col = Vector4f( 0,0,0,a );
        else if( flags & Gray )   col = Vector4f( 0.5,0.5,0.5,a );

        return col;
    }


    void addGeometry( const std::vector<T> &v, const std::vector<T> &c, uint32_t geometry )
    {
        nyx::vertex_buffer_object<float,unsigned int> vbo;
        m_buffers.push_back( vbo );
        m_buffers.back().configure( 3, 4, 2, geometry );
        m_buffers.back().initVertices( v.data(), v.size()/3 );
        m_buffers.back().initColors( c.data() );

        // update the rotation center
        updateCenter( v );
    }


    void updateCenter( const std::vector<T> &v )
    {
        // update min and max
        for( size_t i=0; i<v.size()/3; i++ )
        {
            for( size_t j=0; j<3; j++ )
            {
                m_min(j) = (v[i*3+j] < m_min(j)) ? v[i*3+j] : m_min(j);
                m_max(j) = (v[i*3+j] > m_max(j)) ? v[i*3+j] : m_max(j);
            }
        }

        // update center
        widget<T>::m_center = static_cast<T>(0.5) * (m_min + m_max);
    }


protected:
    // VBO buffers
    BufferList m_buffers;

    // coordinate system
    std::vector<Vector3> m_cs_vertices;
    std::vector<Vector4> m_cs_colors;

    // min and max coords in volume
    Vector3 m_min;
    Vector3 m_max;
};

} // namespace nyx
