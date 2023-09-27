#ifndef WRAPPER_H
#define WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/generic/matrix.h"
#include "structures/vroom/location.h"
#include "structures/vroom/solution/route.h"
#include "utils/exception.h"

namespace vroom::routing {

struct Matrices {
  Matrix<UserDuration> durations;
  Matrix<UserDistance> distances;

  explicit Matrices(std::size_t n) : durations(n), distances(n){};
};

class Wrapper {

public:
  std::string profile;

  virtual Matrices get_matrices(const std::vector<Location>& locs) const = 0;

  virtual void add_geometry(Route& route) const = 0;

  virtual ~Wrapper() = default;

protected:
  explicit Wrapper(std::string profile) : profile(std::move(profile)) {
  }

  static inline void
  check_unfound(const std::vector<Location>& locs,
                const std::vector<unsigned>& nb_unfound_from_loc,
                const std::vector<unsigned>& nb_unfound_to_loc) {
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
      error_msg += "location [" + std::to_string(locs[error_loc].lon()) + "," +
                   std::to_string(locs[error_loc].lat()) + "]";

      throw RoutingException(error_msg);
    }
  }
};

} // namespace vroom::routing

#endif
