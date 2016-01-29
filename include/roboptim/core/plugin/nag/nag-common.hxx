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

#ifndef ROBOPTIM_CORE_NAG_COMMON_HXX
# define ROBOPTIM_CORE_NAG_COMMON_HXX

# include <string>

# include <boost/mpl/vector.hpp>
# include <boost/variant/apply_visitor.hpp>

# include <nagx04.h>

# include <roboptim/core/sys.hh>
# include <roboptim/core/portability.hh>

# include "roboptim/core/plugin/nag/nag-parameters-updater.hh"
# include "roboptim/core/plugin/nag/nag-common.hh"

namespace roboptim
{
  template <typename T>
  NagSolverCommon<T>::NagSolverCommon (const problem_t& pb)
    : solver_t (pb), fdLog_ (-1)
  {
  }

  template <typename T>
  NagSolverCommon<T>::~NagSolverCommon ()
  {
  }

#define DEFINE_PARAMETER(KEY, DESCRIPTION, VALUE)     \
  do                                                  \
  {                                                   \
    this->parameters_[KEY].description = DESCRIPTION; \
    this->parameters_[KEY].value = VALUE;             \
  } while (0)

  template <typename T>
  void NagSolverCommon<T>::initializeParameters ()
  {
    this->parameters_.clear ();

    // Shared parameters.
    DEFINE_PARAMETER ("max-iterations", "number of iterations", 3000);

    // NAG specific.

    //  Output
    DEFINE_PARAMETER ("nag.print-file", "file descriptor", 0);
    DEFINE_PARAMETER ("nag.verify-level", "verify level", 3);

    // Not standard NAG parameter
    DEFINE_PARAMETER ("nag.output_file", "log filename", std::string (""));
  }

#undef DEFINE_PARAMETER

  template <typename T>
  void NagSolverCommon<T>::updateParameters (Nag_E04State* state,
                                             NagError* fail)
  {
    const std::string prefix = "nag.";
    typedef const std::pair<const std::string, Parameter> const_iterator_t;
    BOOST_FOREACH (const_iterator_t& it, this->parameters_)
    {
      if (it.first.substr (0, prefix.size ()) == prefix)
      {
        boost::apply_visitor (
            NagParametersUpdater (it.first.substr (prefix.size ()), state,
                                  fail),
            it.second.value);
      }
    }

    // Remap standardized parameters.
    boost::apply_visitor (
        NagParametersUpdater ("Major Iterations Limit", state, fail),
        this->parameters_["max-iterations"].value);

    // If the user specified a log filename
    typename solver_t::parameters_t::const_iterator it =
        this->parameters_.find ("nag.output_file");
    if (it != this->parameters_.end ())
    {
      std::string filename = boost::get<std::string> (it->second.value);
      if (!filename.empty ())
      {
        if (fdLog_ > 2)
        {
          nag_close_file (fdLog_, fail);
          fdLog_ = -1;
        }

        // 1: open file for writing
        nag_open_file (filename.c_str (), 1, &fdLog_, fail);

        NagParametersUpdater updater ("Print file", state, fail);
        updater (int(fdLog_));
      }
    }
  }
} // end of namespace roboptim

#endif //! ROBOPTIM_CORE_NAG_COMMON_HXX
