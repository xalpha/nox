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

cmake_minimum_required(VERSION 2.8)
project(Nox)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# configure the project
include( NoxConfig.cmake )

# add the include files
list( APPEND Nox_INC
    include/nox/vis.hpp
    include/nox/widget.hpp )

# set include directories
include_directories( ${Nox_INCLUDE_DIRS} )

# add the target
add_custom_target( ${Nox_TARGET} SOURCES ${Nox_INC} )

# install
install(FILES ${Nox_INC} DESTINATION "include/nox" )
install(FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/FindNox.cmake" DESTINATION share )
