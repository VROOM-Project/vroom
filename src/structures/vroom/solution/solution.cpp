/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "solution.h"

solution::solution(unsigned code, std::string error)
  : code(code), error(error), summary(0) {
}

solution::solution(unsigned code, std::vector<route_t>&& routes, cost_t cost)
  : code(code), routes(std::move(routes)), summary(cost) {
}
