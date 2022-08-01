#ifndef SOLUTION_INDICATORS_H
#define SOLUTION_INDICATORS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "utils/helpers.h"

namespace vroom {
namespace utils {

template <class Route> struct SolutionIndicators {
  Priority priority_sum;
  unsigned assigned;
  Cost cost;
  unsigned used_vehicles;

  SolutionIndicators()
    : priority_sum(0), assigned(0), cost(0), used_vehicles(0) {
  }

  SolutionIndicators(const Input& input, const std::vector<Route>& sol)
    : SolutionIndicators() {
    priority_sum =
      std::accumulate(sol.begin(), sol.end(), 0, [&](auto sum, const auto& r) {
        return sum + utils::priority_sum_for_route(input, r.route);
      });

    assigned =
      std::accumulate(sol.begin(), sol.end(), 0, [&](auto sum, const auto& r) {
        return sum + r.route.size();
      });

    Index v_rank = 0;
    cost =
      std::accumulate(sol.begin(), sol.end(), 0, [&](auto sum, const auto& r) {
        return sum + utils::route_cost_for_vehicle(input, v_rank++, r.route);
      });

    used_vehicles = std::count_if(sol.begin(), sol.end(), [](const auto& r) {
      return !r.empty();
    });
  }

  friend bool operator<(const SolutionIndicators& lhs,
                        const SolutionIndicators& rhs) {
    if (lhs.priority_sum > rhs.priority_sum) {
      return true;
    }
    if (lhs.priority_sum == rhs.priority_sum) {
      if (lhs.assigned > rhs.assigned) {
        return true;
      }
      if (lhs.assigned == rhs.assigned) {
        if (lhs.cost < rhs.cost) {
          return true;
        }
        if (lhs.cost == rhs.cost and lhs.used_vehicles < rhs.used_vehicles) {
          return true;
        }
      }
    }

    return false;
  }
};

} // namespace utils
} // namespace vroom

#endif
