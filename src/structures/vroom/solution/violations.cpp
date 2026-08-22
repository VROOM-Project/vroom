/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>

#include "structures/vroom/solution/violations.h"

namespace vroom {

Violations::Violations() : lead_time(0), delay(0), transit_time_excess(0) {
}

Violations::Violations(const UserDuration lead_time,
                       const UserDuration delay,
                       std::unordered_set<VIOLATION>&& types,
                       const UserDuration transit_time_excess)
  : lead_time(lead_time),
    delay(delay),
    transit_time_excess(transit_time_excess),
    types(std::move(types)) {
}

Violations& Violations::operator+=(const Violations& rhs) {
  this->lead_time += rhs.lead_time;
  this->delay += rhs.delay;
  this->transit_time_excess += rhs.transit_time_excess;

  for (const auto t : rhs.types) {
    this->types.insert(t);
  }

  return *this;
}

} // namespace vroom
