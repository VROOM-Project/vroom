#ifndef VIOLATIONS_H
#define VIOLATIONS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom {

struct Violations {
  UserDuration lead_time;
  UserDuration delay;

  std::unordered_set<VIOLATION> types;

  // Used for steps.
  Violations();

  // Used for route/summary.
  Violations(
    UserDuration lead_time,
    UserDuration delay,
    std::unordered_set<VIOLATION>&& types = std::unordered_set<VIOLATION>());

  Violations& operator+=(const Violations& rhs);
};

} // namespace vroom

#endif
