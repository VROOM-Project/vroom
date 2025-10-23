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
  Violations() : lead_time(0), delay(0) {
  }

  // Used for route/summary.
  Violations(
    UserDuration lead_time,
    UserDuration delay,
    std::unordered_set<VIOLATION>&& types = std::unordered_set<VIOLATION>())
    : lead_time(lead_time), delay(delay), types(std::move(types)) {
  }

  Violations& operator+=(const Violations& rhs) {
    this->lead_time += rhs.lead_time;
    this->delay += rhs.delay;

    for (const auto t : rhs.types) {
      this->types.insert(t);
    }
    return *this;
  }
};

} // namespace vroom

#endif
