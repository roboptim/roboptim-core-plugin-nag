# Copyright 2013, Thomas Moulard, CNRS JRL UMI3218
#
# This file is part of roboptim-core.
# roboptim-core is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# roboptim-core is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Lesser Public License for more details.
# You should have received a copy of the GNU Lesser General Public License
# along with roboptim-core.  If not, see <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME roboptim-core-plugin-nag)
SET(PROJECT_DESCRIPTION "RobOptim core NAG plug-in")
SET(PROJECT_URL "https://github.com/roboptim/roboptim-core-plugin-nag")

SET(HEADERS
  ${CMAKE_SOURCE_DIR}/include/roboptim/core/plugin/nag/nag.hh
  ${CMAKE_SOURCE_DIR}/include/roboptim/core/plugin/nag/nag-differentiable.hh
  ${CMAKE_SOURCE_DIR}/include/roboptim/core/plugin/nag/nag-nlp.hh
  ${CMAKE_SOURCE_DIR}/include/roboptim/core/plugin/nag/nag-nlp-sparse.hh
  ${CMAKE_SOURCE_DIR}/include/roboptim/core/plugin/nag/nag-simplex.hh
  )

SET(PKG_CONFIG_ADDITIONAL_VARIABLES plugindir ${PKG_CONFIG_ADDITIONAL_VARIABLES})

SETUP_PROJECT()

# If compiler support symbol visibility, enable it.
INCLUDE(CheckCCompilerFlag)
CHECK_C_COMPILER_FLAG(-fvisibility=hidden HAS_VISIBILITY)
IF(HAS_VISIBILITY)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
ENDIF()

# Search for dependencies.
SET(BOOST_COMPONENTS
  date_time filesystem system thread program_options unit_test_framework)
SEARCH_FOR_BOOST()

# Libtool dynamic loading
# This project does not use Libtool directly but still uses ltdl for
# plug-in loading.
SET(LIB_LTDL LIB_LTDL-NOTFOUND)
FIND_LIBRARY(LIB_LTDL libltdl${CMAKE_SHARED_LIBRARY_SUFFIX})
IF (NOT LIB_LTDL)
  MESSAGE(FATAL_ERROR
    "Failed to find Libtool ltdl library, check that Libtool ltdl is installed.")
ENDIF()
INCLUDE(CheckIncludeFileCXX)
CHECK_INCLUDE_FILE_CXX(ltdl.h LTDL_H_FOUND)
IF (NOT LTDL_H_FOUND)
  MESSAGE(FATAL_ERROR
    "Failed to find ltdl.h, check that Libtool ltdl is installed.")
ENDIF()

# Search for roboptim-core.
ADD_REQUIRED_DEPENDENCY("roboptim-core >= 3.2")

# Look for NAG.
FIND_PATH(NAG_DIR include/nag.h HINTS
  /opt/NAG/cll6a23dhl
  /opt/NAG/cll6i24dcl
  /opt/NAG/cll6i25dcl)
MESSAGE(STATUS "NAG_DIR: " ${NAG_DIR})

INCLUDE_DIRECTORIES("${NAG_DIR}/include")
LINK_DIRECTORIES("${NAG_DIR}/lib")
LINK_DIRECTORIES("${NAG_DIR}/rtl/intel64")

# Enable SIGFPE signal to detect arithmetic errors in tests
OPTION(ENABLE_SIGFPE "Enable floating-point exceptions" OFF)
IF(ENABLE_SIGFPE)
  ADD_DEFINITIONS(-DENABLE_SIGFPE)
ENDIF()

OPTION(DISABLE_TESTS "Disable test programs" OFF)

ADD_SUBDIRECTORY(src)

IF(NOT DISABLE_TESTS)
  ADD_SUBDIRECTORY(tests)
ELSE()
  MESSAGE(WARNING
    "Tests should only be disabled for speficic cases. Do it at your own risk.")
ENDIF()

HEADER_INSTALL("${HEADERS}")

SETUP_PROJECT_FINALIZE()
