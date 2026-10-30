#ifndef SOFT_SKILL_PREFERENCE_H
#define SOFT_SKILL_PREFERENCE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cstddef>

namespace vroom {

// Aggregated soft-skill preference counts reported for a solution
// route or at summary level. By construction, a summary-level value
// aggregates the route-level ones and, for any single route,
// eligible == satisfied + unsatisfied holds.
struct SoftSkillPreference {
  std::size_t eligible{0};
  std::size_t satisfied{0};
  std::size_t unsatisfied{0};

  SoftSkillPreference& operator+=(const SoftSkillPreference& rhs) {
    eligible += rhs.eligible;
    satisfied += rhs.satisfied;
    unsatisfied += rhs.unsatisfied;
    return *this;
  }
};

} // namespace vroom

#endif
