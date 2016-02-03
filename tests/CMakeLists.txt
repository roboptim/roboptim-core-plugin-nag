# Copyright 2013, Thomas Moulard, CNRS-AIST.
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

# Include shared test suite.
SET(PLUGIN_PATH "${CMAKE_BINARY_DIR}/src")
INCLUDE(shared-tests/tests.cmake)

SET(SOLVER_NAME "nag")
SET(FUNCTION_TYPE ::roboptim::EigenMatrixDense)
SET(PROGRAM_SUFFIX "")
SET(COST_FUNCTION_TYPE ::roboptim::GenericDifferentiableFunction)
SET(CONSTRAINT_TYPE_1 ::roboptim::GenericLinearFunction)
SET(CONSTRAINT_TYPE_2 ::roboptim::GenericDifferentiableFunction)
BUILD_COMMON_TESTS()
BUILD_TEST("roboptim/simple-square")

SET(SCHITTKOWSKI_PROBLEMS_FAIL
  2 8
  13 14 16
  20 23 25 28
  33
  47
  55 56
  72)

SET(SOLVER_NAME "nag-nlp")
SET(FUNCTION_TYPE ::roboptim::EigenMatrixDense)
SET(PROGRAM_SUFFIX "-nlp")
SET(COST_FUNCTION_TYPE ::roboptim::GenericDifferentiableFunction)
SET(CONSTRAINT_TYPE_1 ::roboptim::GenericLinearFunction)
SET(CONSTRAINT_TYPE_2 ::roboptim::GenericDifferentiableFunction)
BUILD_COMMON_TESTS()
BUILD_SCHITTKOWSKI_PROBLEMS()
BUILD_QP_PROBLEMS()
BUILD_ROBOPTIM_PROBLEMS()

SET(SCHITTKOWSKI_PROBLEMS_FAIL ${SCHITTKOWSKI_PROBLEMS_FAIL}
  24 44)

SET(SOLVER_NAME "nag-nlp-sparse")
SET(FUNCTION_TYPE ::roboptim::EigenMatrixSparse)
SET(PROGRAM_SUFFIX "-nlp-sparse")
SET(COST_FUNCTION_TYPE ::roboptim::GenericDifferentiableFunction)
SET(CONSTRAINT_TYPE_1 ::roboptim::GenericLinearFunction)
SET(CONSTRAINT_TYPE_2 ::roboptim::GenericDifferentiableFunction)
BUILD_COMMON_TESTS()
BUILD_SCHITTKOWSKI_PROBLEMS()
BUILD_QP_PROBLEMS()
BUILD_ROBOPTIM_PROBLEMS()
