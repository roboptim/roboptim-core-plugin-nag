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

#include <cassert>
#include <cstring>

#include <roboptim/core/function.hh>

#include <nag.h>
#include <nage04.h>

#include <roboptim/core/plugin/nag/nag-simplex.hh>

#define DEFINE_PARAMETER(KEY, DESCRIPTION, VALUE)     \
  do                                                  \
  {                                                   \
    this->parameters_[KEY].description = DESCRIPTION; \
    this->parameters_[KEY].value = VALUE;             \
  } while (0)

namespace roboptim
{
  namespace nag
  {
    namespace detail
    {
      static void solverCallback (Integer ROBOPTIM_DEBUG_ONLY (n),
                                  const double xc[], double* fc, Nag_Comm* comm)
      {
        assert (!!comm);
        assert (!!comm->p);
        Simplex* solver = static_cast<Simplex*> (comm->p);
        assert (!!solver);
        assert (n == solver->problem ().function ().inputSize ());

        Eigen::Map<const Function::vector_t> x_ (
          xc, solver->problem ().function ().inputSize ());
        Eigen::Map<Function::vector_t> fc_ (
          fc, solver->problem ().function ().outputSize ());

        fc_.setZero ();
        fc_ = solver->problem ().function () (x_);

        if (!solver->callback ()) return;
        solver->solverState ().x () = x_;
        solver->callback () (solver->problem (), solver->solverState ());
      }
    } // end of namespace detail

    Simplex::Simplex (const problem_t& pb)
      : parent_t (pb),
        x_ (problem ().function ().inputSize ()),
        f_ (problem ().function ().outputSize ()),
        callback_ (),
        solverState_ (pb)
    {
      x_.setZero ();
      f_.setZero ();

      // Shared parameters.
      DEFINE_PARAMETER ("max-iterations", "number of iterations", 3000);

      // Custom parameters
      DEFINE_PARAMETER ("nag.tolx",
                        "the error tolerable in the spatial values",
                        Function::epsilon ());
      DEFINE_PARAMETER ("nag.tolf",
                        "the error tolerable in the function values",
                        Function::epsilon ());
    }

    Simplex::~Simplex ()
    {
    }

    void Simplex::solve ()
    {
      // Solution.
      if (problem ().startingPoint ()) x_ = *(problem ().startingPoint ());

      // Nag communication object.
      Nag_Comm comm;
      std::memset (&comm, 0, sizeof (Nag_Comm));
      comm.p = this;

      // Nag error code.
      NagError fail;
      std::memset (&fail, 0, sizeof (NagError));
      INIT_FAIL (fail);

      // Solver options.
      double tolx = boost::get<double> (this->parameters_["nag.tolx"].value);
      double tolf = boost::get<double> (this->parameters_["nag.tolf"].value);
      int max_iter =
        boost::get<int> (this->parameters_["max-iterations"].value);

      nag_opt_simplex_easy (problem ().function ().inputSize (),
                            x_.data (), f_.data (), tolf, tolx,
                            &detail::solverCallback, NULL, max_iter,
                            &comm, &fail);


      if (fail.code == NE_NOERROR)
      {
        Result res (problem ().function ().inputSize (),
                    problem ().function ().outputSize ());
        res.x = x_;
        res.value = f_;
        result_ = res;
        return;
      }

      this->result_ = SolverError (fail.message);
    }
  } // end of namespace nag.
} // end of namespace roboptim.

extern "C" {
  typedef roboptim::nag::Simplex Simplex;
  typedef roboptim::Solver<roboptim::EigenMatrixDense> solver_t;

  ROBOPTIM_DLLEXPORT unsigned getSizeOfProblem ();
  ROBOPTIM_DLLEXPORT const char* getTypeIdOfConstraintsList ();
  ROBOPTIM_DLLEXPORT solver_t* create (const Simplex::problem_t& pb);
  ROBOPTIM_DLLEXPORT void destroy (solver_t* p);

  ROBOPTIM_DLLEXPORT unsigned getSizeOfProblem ()
  {
    return sizeof (Simplex::problem_t);
  }

  ROBOPTIM_DLLEXPORT const char* getTypeIdOfConstraintsList ()
  {
    return typeid (Simplex::problem_t::constraintsList_t).name ();
  }

  ROBOPTIM_DLLEXPORT solver_t* create (const Simplex::problem_t& pb)
  {
    return new roboptim::nag::Simplex (pb);
  }

  ROBOPTIM_DLLEXPORT void destroy (solver_t* p)
  {
    delete p;
  }
}
