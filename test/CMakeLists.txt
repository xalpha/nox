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

# find glut
find_package( GLUT QUIET )

# if GLUT found
if( GLUT_FOUND )

    # set include directories
    include_directories( ${Nox_INCLUDE_DIRS} ${GLUT_INCLUDE_DIR} )

    # add test for widget
    set( Nox_Test_widget test_widget )
    add_executable( ${Nox_Test_widget} test_widget.cpp )
    target_link_libraries( ${Nox_Test_widget} -lm -lc -Wall ${Nox_LIBRARIES} ${GLUT_LIBRARIES} )
    add_test( ${Nox_Test_widget} ${Nox_Test_widget} )

    # add test for plot
    #set( Nox_Test_plot test_plot )
    #add_executable( ${Nox_Test_plot} test_plot.cpp )
    #target_link_libraries( ${Nox_Test_plot} -lm -lc -Wall ${Nox_LINK_LIBRARIES} )
    #add_test( ${Nox_Test_plot} ${Nox_Test_plot} )

elseif()
    message( WARNING "GLUT not found, tests disabled." )
endif()


