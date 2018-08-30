/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/solution.h"

solution::solution(unsigned code, std::string error)
  : code(code), error(error) {
}

solution::solution(unsigned code,
                   unsigned amount_size,
                   std::vector<route_t>&& routes,
                   std::vector<job_t>&& unassigned)
  : code(code),
    summary(unassigned.size(), amount_size),
    routes(std::move(routes)),
    unassigned(std::move(unassigned)) {

  for (const auto& route : this->routes) {
    summary.cost += route.cost;
    summary.amount += route.amount;
    summary.service += route.service;
    summary.duration += route.duration;
    summary.waiting_time += route.waiting_time;
  }
}
