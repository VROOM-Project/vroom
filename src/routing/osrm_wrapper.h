#ifndef OSRM_WRAPPER_H
#define OSRM_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "routing/routing_io.h"
#include "structures/abstract/matrix.h"
#include "utils/exceptions.h"

class osrm_wrapper : public routing_io<cost_t> {

protected:
  const std::string _osrm_profile; // OSRM profile name

  static cost_t round_cost(double value) {
    return static_cast<cost_t>(value + 0.5);
  }

  osrm_wrapper(const std::string& osrm_profile) : _osrm_profile(osrm_profile) {
  }

  inline void
  check_unfound(const std::vector<location_t>& locs,
                const std::vector<unsigned>& nb_unfound_from_loc,
                const std::vector<unsigned>& nb_unfound_to_loc) const {
    assert(nb_unfound_from_loc.size() == nb_unfound_to_loc.size());
    unsigned max_unfound_routes_for_a_loc = 0;
    unsigned error_loc = 0; // Initial value never actually used.
    std::string error_direction;
    // Finding the "worst" location for unfound routes.
    for (unsigned i = 0; i < nb_unfound_from_loc.size(); ++i) {
      if (nb_unfound_from_loc[i] > max_unfound_routes_for_a_loc) {
        max_unfound_routes_for_a_loc = nb_unfound_from_loc[i];
        error_loc = i;
        error_direction = "from";
      }
      if (nb_unfound_to_loc[i] > max_unfound_routes_for_a_loc) {
        max_unfound_routes_for_a_loc = nb_unfound_to_loc[i];
        error_loc = i;
        error_direction = "to";
      }
    }
    if (max_unfound_routes_for_a_loc > 0) {
      std::string error_msg = "OSRM has unfound route(s) ";
      error_msg += error_direction;
      error_msg += "location [" + std::to_string(locs[error_loc].lon()) + ";" +
                   std::to_string(locs[error_loc].lat()) + "]";

      throw custom_exception(error_msg);
    }
  }

  virtual matrix<cost_t>
  get_matrix(const std::vector<location_t>& locs) const = 0;

  virtual void add_route_info(route_t& route) const = 0;
};

#endif
