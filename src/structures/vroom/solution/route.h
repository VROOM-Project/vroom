#ifndef ROUTE_H
#define ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/vroom/solution/step.h"
#include "structures/vroom/solution/violations.h"

namespace vroom {

struct Route {
  Id vehicle;
  std::vector<Step> steps;
  UserCost cost;
  UserDuration duration;
  UserDistance distance;
  UserDuration setup;
  UserDuration service;
  UserDuration waiting_time;
  Priority priority;
  Amount delivery;
  Amount pickup;
  std::string profile;
  std::string description;
  Violations violations;
  
  // Cargo lifetime tracking
  UserDuration max_cargo_age{0};
  unsigned lifetime_violations{0};

  std::string geometry;

  Route();

  Route(Id vehicle,
        std::vector<Step>&& steps,
        UserCost cost,
        UserDuration duration,
        UserDistance distance,
        UserDuration setup,
        UserDuration service,
        UserDuration waiting_time,
        Priority priority,
        Amount delivery,
        Amount pickup,
        std::string profile,
        std::string description,
        Violations&& violations = Violations(0, 0));

  void check_timing_consistency() const;
  
  // Cargo lifetime methods
  void update_cargo_lifetime_stats();
  
  bool has_lifetime_violations() const {
    return lifetime_violations > 0;
  }
};

} // namespace vroom

#endif
