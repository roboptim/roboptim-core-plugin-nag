// Copyright (C) 2013 by Thomas Moulard, AIST, CNRS.
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

#ifndef ROBOPTIM_CORE_PLUGIN_NAG_NAG_DIFFERENTIABLE_HH
# define ROBOPTIM_CORE_PLUGIN_NAG_NAG_DIFFERENTIABLE_HH

# include <vector>

# include <roboptim/core/solver.hh>
# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  /// \addtogroup roboptim_solver
  /// @{

  /// \brief Solver for C1 function with gradient computation, no
  ///        constraint.
  ///
  /// Search for a minimum, in a given finite interval, of a
  /// continuous function of a single variable, using function and
  /// first derivative values. The method (based on cubic
  /// interpolation) is intended for functions which have a continuous
  /// first derivative (although it will usually work if the
  /// derivative has occasional discontinuities).
  ///
  /// \see http://www.nag.com/numeric/CL/nagdoc_cl23/html/E04/e04bbc.html
  class ROBOPTIM_DLLEXPORT NagSolverDifferentiable
    : public Solver<EigenMatrixDense>
  {
  public:
    typedef Solver<EigenMatrixDense> parent_t;
    typedef Function::argument_t argument_t;
    typedef Function::result_t result_t;
    typedef DifferentiableFunction::gradient_t gradient_t;

    explicit NagSolverDifferentiable (const problem_t& pb);
    virtual ~NagSolverDifferentiable ();

    /// \brief Solve the problem.
    void solve ();

    void setIterationCallback (callback_t callback)
    {
      callback_ = callback;
    }

    const callback_t& callback () const
    {
      return callback_;
    }

    solverState_t& solverState ()
    {
      return this->solverState_;
    }

  private:
    /// \brief Relative accuracy.
    double e1_;
    /// \brief Absolute accuracy.
    double e2_;
    /// \brief Lower bound.
    std::vector<double> a_;
    /// \brief Upper bound.
    std::vector<double> b_;
    /// \brief Current minimum estimation.
    argument_t x_;
    /// \brief Current cost.
    result_t f_;
    /// \brief Current gradient.
    gradient_t g_;

    /// \brief Per-iteration callback function.
    callback_t callback_;

    /// \brief Solver state
    solverState_t solverState_;
  };

  /// @}
} // end of namespace roboptim

#endif //! ROBOPTIM_CORE_PLUGIN_NAG_NAG_DIFFERENTIABLE_HH
