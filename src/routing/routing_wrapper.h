#ifndef ROUTING_WRAPPER_H
#define ROUTING_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "routing/wrapper.h"
#include "structures/generic/matrix.h"
#include "utils/exception.h"

namespace vroom {
namespace routing {

class RoutingWrapper : public Wrapper<Cost> {

protected:
  static Cost round_cost(double value) {
    return static_cast<Cost>(value + 0.5);
  }

  RoutingWrapper(const std::string& profile) : Wrapper(profile) {
  }

  inline void
  check_unfound(const std::vector<Location>& locs,
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
        error_direction = "from ";
      }
      if (nb_unfound_to_loc[i] > max_unfound_routes_for_a_loc) {
        max_unfound_routes_for_a_loc = nb_unfound_to_loc[i];
        error_loc = i;
        error_direction = "to ";
      }
    }
    if (max_unfound_routes_for_a_loc > 0) {
      std::string error_msg = "Unfound route(s) ";
      error_msg += error_direction;
      error_msg += "location [" + std::to_string(locs[error_loc].lon()) + ";" +
                   std::to_string(locs[error_loc].lat()) + "]";

      throw Exception(ERROR::ROUTING, error_msg);
    }
  }

  virtual Matrix<Cost> get_matrix(const std::vector<Location>& locs) const = 0;

  virtual void add_route_info(Route& route) const = 0;
};

} // namespace routing
} // namespace vroom

#endif
