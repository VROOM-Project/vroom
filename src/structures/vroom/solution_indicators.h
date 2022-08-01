#ifndef SOLUTION_INDICATORS_H
#define SOLUTION_INDICATORS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"

namespace vroom {
namespace utils {

struct SolutionIndicators {
  Priority priority_sum;
  unsigned assigned;
  Cost cost;
  unsigned used_vehicles;

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
