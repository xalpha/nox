##############################################################################
#                                                                            #
# This file is part of nox, a lightweight C++ template visualization library #
#                                                                            #
# Copyright (C) 2012 Alexandru Duliu                                         #
#                                                                            #
# nox is free software; you can redistribute it and/or                       #
# modify it under the terms of the GNU Lesser General Public                 #
# License as published by the Free Software Foundation; either               #
# version 3 of the License, or (at your option) any later version.           #
#                                                                            #
# nox is distributed in the hope that it will be useful, but WITHOUT ANY     #
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS  #
# FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License or the #
# GNU General Public License for more details.                               #
#                                                                            #
# You should have received a copy of the GNU Lesser General Public           #
# License along with nox. If not, see <http://www.gnu.org/licenses/>.        #
#                                                                            #
##############################################################################

# Config file for the nox library
# It defines the following variables
#
# Nox_INCLUDE_DIR - include directory for nox headers
# Nox_INCLUDE_DIRS - all include directories nox needs
# Nox_LIBRARIES - all include directories nox needs

# set path
set( Nox_DIR ${CMAKE_CURRENT_LIST_DIR})
set( ENV{Nox_DIR} ${Nox_DIR} )

# add module paths
list( APPEND CMAKE_MODULE_PATH ${Nox_DIR}/cmake ${CMAKE_INSTALL_PREFIX}/share )

# set the include dir
set( Nox_INCLUDE_DIR "${Nox_DIR}/include")

# set target names
set( Nox_TARGET nox )

# set compile definitions
set( Nox_COMPILE_DEFINITIONS NOX CACHE INTERNAL "all compile definitions nox needs"  )

# set linker flags
if( WIN32 )
    list( APPEND Nox_LINK_FLAGS " /MANIFEST:NO" )
endif()

# find Eigen3
find_package( Eigen3 REQUIRED )

# set include directories
set( Nox_INCLUDE_DIRS
    ${Nox_INCLUDE_DIR}
    ${Nox_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR} CACHE INTERNAL "all include directories nox needs" )

# link libraries
set( Nox_LIBRARIES ${NYX_LIBRARIRES} CACHE INTERNAL "all libs nox needs" )

# enable C++11 support
#if( NOT WIN32 )
#    if( CMAKE_COMPILER_IS_GNUCXX )
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++0x")
#    else( CMAKE_COMPILER_IS_GNUCXX )
#        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Qunused-arguments")
#    endif()
#endif()



