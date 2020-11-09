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

namespace vroom {

struct TimingViolations {
  const Duration start_lead_time;
  const Duration lead_time;
  const Duration end_delay;
  const Duration delay;

  TimingViolations()
    : start_lead_time(0), lead_time(0), end_delay(0), delay(0) {
  }

  TimingViolations(const Duration start_lead_time,
                   const Duration lead_time,
                   const Duration end_delay,
                   const Duration delay)
    : start_lead_time(start_lead_time),
      lead_time(lead_time),
      end_delay(end_delay),
      delay(delay) {
  }
};

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
