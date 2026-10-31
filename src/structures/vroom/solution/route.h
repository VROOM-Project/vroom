#ifndef ROUTE_H
#define ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <optional>
#include <string>

#include "structures/vroom/solution/soft_skill_preference.h"
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

  // Only set when the input holds soft-skills, then reported in
  // output_json as "soft_skill_preference". Authoritative counts from
  // ls::compute_route_preference (Task 25).
  std::optional<SoftSkillPreference> soft_skill_preference;

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
};

} // namespace vroom

#endif
