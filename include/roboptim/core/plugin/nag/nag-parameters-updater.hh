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

#ifndef ROBOPTIM_CORE_NAG_PARAMETERS_UPDATER_HH
# define ROBOPTIM_CORE_NAG_PARAMETERS_UPDATER_HH

# include <map>
# include <string>
# include <set>
# include <algorithm>

# include <boost/variant/static_visitor.hpp>
# include <boost/static_assert.hpp>

# include <nag.h>
# include <nage04.h>

namespace roboptim
{
  struct NagParametersUpdater : public boost::static_visitor<>
  {
    NagParametersUpdater (const std::string& key, Nag_E04State* state,
                          NagError* fail)
      : boost::static_visitor<> (),
        key_ (),
        state_ (state),
        fail_ (fail),
        ignored_ ()
    {
      // Conversion map from RobOptim whitespace-free strings to
      // NAG's strings
      toRoboptim_["max-iterations"] = std::string ("Major Iterations Limit");
      toRoboptim_["verify-level"] = std::string ("Verify Level");
      toRoboptim_["print-file"] = std::string ("Print file");
      key_ = roboptim_to_nag (key);

      ignored_.insert ("output_file");
    }

    void operator() (const Function::value_type& val) const
    {
      if (is_ignored (key_)) return;

      nag_opt_sparse_nlp_option_set_double (key_.c_str (), val, state_, fail_);
    }

    void operator() (const int& val) const
    {
      if (is_ignored (key_)) return;

      nag_opt_sparse_nlp_option_set_integer (key_.c_str (), val, state_, fail_);
    }

    void operator() (const std::string& val) const
    {
      if (is_ignored (key_)) return;

      std::string option = key_;
      if (!val.empty ()) option += " = " + val;
      nag_opt_sparse_nlp_option_set_string (option.c_str (), state_, fail_);
    }

    void operator() (const char* val) const
    {
      if (is_ignored (key_)) return;

      (*this) (std::string (val));
    }

    template <typename T>
    void operator() (const T&) const
    {
      ROBOPTIM_ASSERT_MSG (false, "NOT IMPLEMENTED");
    }

  private:
    bool is_ignored (const std::string& option) const
    {
      return std::find (ignored_.begin (), ignored_.end (), option) !=
             ignored_.end ();
    }

    std::string roboptim_to_nag (const std::string& option) const
    {
      std::map<std::string, std::string>::const_iterator it =
          toRoboptim_.find (option);
      if (it != toRoboptim_.end ())
      {
        // element found
        return it->second;
      }
      else
        return option;
    }

  private:
    std::string key_;
    Nag_E04State* state_;
    NagError* fail_;
    std::set<std::string> ignored_;
    std::map<std::string, std::string> toRoboptim_;
  };
} // end of namespace roboptim.

#endif //! ROBOPTIM_CORE_NAG_PARAMETERS_UPDATER_HH
