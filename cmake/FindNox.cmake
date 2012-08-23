##############################################################################
#                                                                            #
# This file is part of nox, a lightweight C++ template library for OpenGL    #
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

#  Try to find Nox
#
#  Nox_FOUND - System has Nox
#  Nox_INCLUDE_DIRS - The Nox include directories
#  Nox_LIBRARIES - All libraries Nox needs


# try to find the include dir
find_path( Nox_INCLUDE_DIR 
    NAMES
        nox/widget.hpp
    PATHS
	    $ENV{Nox_DIR}/include
	    %{CMAKE_INSTALL_PREFIX}/include
        /usr/include
        /usr/local/include
        /opt/include
        /opt/local/include
    PATH_SUFFIXES
        nox )

# check if this is a valid component
if( Nox_INCLUDE_DIR )
    # include the component
    MESSAGE( STATUS "Nox found.")
else()
    MESSAGE( FATAL_ERROR "Nox target not available.")
endif()

# set the include dirs
set( Nox_INCLUDE_DIRS 
    ${Nox_INCLUDE_DIR}
    ${Nox_INCLUDE_DIR}/nox )
    
    
#####
## Dependencies
###

# find Nyx
if( NOT Nyx_FOUND )
    find_package( Nyx REQUIRED )
endif()
list( APPEND Nox_INCLUDE_DIRS ${Nyx_INCLUDE_DIRS} )
list( APPEND Nox_LIBRARIES ${Nyx_LIBRARIES} )

