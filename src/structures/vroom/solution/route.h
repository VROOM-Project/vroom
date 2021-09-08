#ifndef ROUTE_H
#define ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/vroom/solution/step.h"
#include "structures/vroom/solution/violations.h"

namespace vroom {

struct Route {
  Id vehicle;
  std::vector<Step> steps;
  Cost cost;
  Duration setup;
  Duration service;
  Duration duration;
  Duration waiting_time;
  Priority priority;
  Amount delivery;
  Amount pickup;
  std::string profile;
  std::string description;
  Violations violations;

  std::string geometry;
  Distance distance;

  Route();

  Route(Id vehicle,
        std::vector<Step>&& steps,
        Cost cost,
        Duration setup,
        Duration service,
        Duration duration,
        Duration waiting_time,
        Priority priority,
        const Amount& delivery,
        const Amount& pickup,
        const std::string& profile,
        const std::string& description,
        const Violations&& violations = Violations(0, 0));
};

} // namespace vroom

#endif
