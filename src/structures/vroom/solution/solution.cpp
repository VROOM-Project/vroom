/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "solution.h"

solution::solution(unsigned code, std::string error)
  : code(code), error(error), summary(0, 0) {
}

solution::solution(unsigned code,
                   cost_t cost,
                   std::vector<route_t>&& routes,
                   std::unordered_set<job_t>&& unassigned)
  : code(code),
    summary(cost, unassigned.size()),
    routes(std::move(routes)),
    unassigned(std::move(unassigned)) {
}
