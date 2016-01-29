// Copyright (C) 2016 by Benjamin Chrétien, CNRS-AIST JRL.
//
// This file is part of the roboptim.
//
// roboptim is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim.  If not, see <http://www.gnu.org/licenses/>.

#ifndef ROBOPTIM_CORE_NAG_COMMON_HH
# define ROBOPTIM_CORE_NAG_COMMON_HH

# include <fstream>

# include <nag.h>
# include <nage04.h>

# include <roboptim/core/sys.hh>
# include <roboptim/core/portability.hh>
# include <roboptim/core/solver.hh>
# include <roboptim/core/linear-function.hh>
# include <roboptim/core/differentiable-function.hh>
# include <roboptim/core/twice-differentiable-function.hh>

namespace roboptim
{
  /// \brief Error handler for NAG API.
  /// \param s error message.
  /// \param code error code.
  /// \param name error name.
  void errorHandler (const char* s, int code, const char* name);

  /// \addtogroup roboptim_problem
  /// @{

  /// \brief NAG common solver.
  ///
  /// This solver shares common piece of code of the different NAG solvers.
  template <typename T>
  class ROBOPTIM_DLLEXPORT NagSolverCommon : public Solver<T>
  {
  public:
    /// \brief Categorize constraints.
    ///
    /// Used with the which method of the Boost.Variant, it
    /// allows to check for a constraint's real type.
    ///
    /// \warning Make sure to keep enum values in the
    /// same order than the MPL vector used to specify CLIST.
    enum ConstraintType
    {
      /// \brief Constraint is a linear function.
      LINEAR = 0,
      /// \brief Constraint is a differentiable or a twice
      /// differentiable function depending on the solve.
      NONLINEAR = 1
    };

    /// \brief Parent type (solver).
    typedef Solver<T> solver_t;

    /// \brief Problem type.
    typedef typename solver_t::problem_t problem_t;

    /// \brief Instantiate the solver from a problem.
    /// \param problem problem that will be solved
    explicit NagSolverCommon (const problem_t& pb);
    virtual ~NagSolverCommon ();

  protected:
    /// \brief Initialize parameters.
    /// Add solver parameters. Called during construction.
    void initializeParameters ();

    /// \brief Read parameters and update associated options in NAG.
    /// Called before solving problem.
    /// \param state internal information required for NAG functions.
    /// \param fail NAG error argument
    void updateParameters (Nag_E04State* state, NagError* fail);

  private:
    /// \brief File descriptor for logging.
    Nag_FileID fdLog_;
  };

  /// @}
} // end of namespace roboptim

# include "roboptim/core/plugin/nag/nag-common.hxx"

#endif //! ROBOPTIM_CORE_NAG_COMMON_HH
