# Copyright 2013, Thomas Moulard, LAAS-CNRS
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

# Define the directory where plug-ins will be installed.
GET_FILENAME_COMPONENT(RELPLUGINDIR ${ROBOPTIM_CORE_PLUGINDIR} NAME)
SET(PLUGINDIR ${CMAKE_INSTALL_LIBDIR}/${RELPLUGINDIR})

MACRO(NAG_PLUGIN NAME)
  ADD_LIBRARY(roboptim-core-plugin-${NAME} MODULE ${NAME}.cc ${HEADERS})
  SET_TARGET_PROPERTIES(roboptim-core-plugin-${NAME} PROPERTIES
    PREFIX ""
    SOVERSION 3 VERSION 3.2.0
    INSTALL_RPATH "${NAG_DIR}/lib")
  INSTALL(TARGETS roboptim-core-plugin-${NAME} DESTINATION ${PLUGINDIR})
  TARGET_LINK_LIBRARIES(roboptim-core-plugin-${NAME} nagc_nag)
  PKG_CONFIG_USE_DEPENDENCY(roboptim-core-plugin-${NAME} roboptim-core)
  PKG_CONFIG_USE_COMPILE_DEPENDENCY(roboptim-core-plugin-${NAME} roboptim-core)
ENDMACRO()

NAG_PLUGIN(nag)
NAG_PLUGIN(nag-differentiable)
NAG_PLUGIN(nag-simplex)
NAG_PLUGIN(nag-nlp)
NAG_PLUGIN(nag-nlp-sparse)
