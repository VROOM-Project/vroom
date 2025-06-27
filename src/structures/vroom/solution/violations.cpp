/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>

#include "structures/vroom/solution/violations.h"

namespace vroom {

Violations::Violations() : lead_time(0), delay(0), lifetime_violations(0) {
}

Violations::Violations(const UserDuration lead_time,
                       const UserDuration delay,
                       std::unordered_set<VIOLATION>&& types,
                       const UserDuration lifetime_violations)
  : lead_time(lead_time), delay(delay), lifetime_violations(lifetime_violations), types(std::move(types)) {
}

Violations& Violations::operator+=(const Violations& rhs) {
  this->lead_time += rhs.lead_time;
  this->delay += rhs.delay;
  this->lifetime_violations += rhs.lifetime_violations;

  for (const auto t : rhs.types) {
    this->types.insert(t);
  }

  return *this;
}

} // namespace vroom
