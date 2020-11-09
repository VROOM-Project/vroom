#ifndef ROUTE_H
#define ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <vector>

#include "structures/vroom/solution/step.h"
#include "structures/vroom/solution/timing_violations.h"

namespace vroom {

struct Route {
  const Id vehicle;
  std::vector<Step> steps;
  const Cost cost;
  const Duration service;
  const Duration duration;
  const Duration waiting_time;
  const Priority priority;
  const Amount delivery;
  const Amount pickup;
  const std::string description;
  const TimingViolations timing_violations;
  const std::unordered_set<VIOLATION> violations;

  std::string geometry;
  Distance distance;

  Route(Id vehicle,
        std::vector<Step>&& steps,
        Cost cost,
        Duration service,
        Duration duration,
        Duration waiting_time,
        Priority priority,
        const Amount& delivery,
        const Amount& pickup,
        const std::string& description,
        const TimingViolations&& timing_violations = TimingViolations(),
        const std::unordered_set<VIOLATION>&& violations =
          std::unordered_set<VIOLATION>());
};

} // namespace vroom

#endif
