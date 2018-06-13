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
                   cost_t cost,
                   std::vector<route_t>&& routes,
                   std::vector<job_t>&& unassigned,
                   duration_t service,
                   amount_t&& amount)
  : code(code),
    summary(cost, unassigned.size(), service, std::move(amount)),
    routes(std::move(routes)),
    unassigned(std::move(unassigned)) {
}
