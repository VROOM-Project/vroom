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
  std::vector<Cost> routes_costs;
  Cost cost;
  unsigned used_vehicles;

  SolutionIndicators(unsigned nb_routes = 0)
    : priority_sum(0),
      assigned(0),
      routes_costs(nb_routes),
      cost(0),
      used_vehicles(0) {
  }

  SolutionIndicators(const Input& input, const std::vector<Route>& sol)
    : SolutionIndicators(sol.size()) {
    Index v_rank = 0;
    for (const auto& r : sol) {
      priority_sum += utils::priority_sum_for_route(input, r.route);
      assigned += r.route.size();

      routes_costs[v_rank] =
        utils::route_cost_for_vehicle(input, v_rank, r.route);
      cost += routes_costs[v_rank];
      ++v_rank;

      if (!r.empty()) {
        used_vehicles += 1;
      }
    }
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
