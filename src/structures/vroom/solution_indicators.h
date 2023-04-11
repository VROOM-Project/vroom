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

namespace vroom::utils {

template <class Route> struct SolutionIndicators {
  Priority priority_sum{0};
  unsigned assigned{0};
  Eval eval;
  unsigned used_vehicles{0};

  SolutionIndicators() = default;

  SolutionIndicators(const Input& input, const std::vector<Route>& sol)
    : SolutionIndicators() {
    Index v_rank = 0;
    for (const auto& r : sol) {
      priority_sum += utils::priority_sum_for_route(input, r.route);
      assigned += r.route.size();

      eval += utils::route_eval_for_vehicle(input, v_rank, r.route);
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
        if (lhs.eval.cost < rhs.eval.cost) {
          return true;
        }
        if (lhs.eval.cost == rhs.eval.cost) {
          if (lhs.eval.duration < rhs.eval.duration) {
            return true;
          }
          if (lhs.eval.duration == rhs.eval.duration and
              lhs.used_vehicles < rhs.used_vehicles) {
            return true;
          }
        }
      }
    }

    return false;
  }
};

} // namespace vroom::utils

#endif
