/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "solution.h"

solution::solution(index_t code, std::string error)
  : code(code),
    error(error),
    summary(0) {}

solution::solution(index_t code, std::vector<route_t>&& routes, duration_t cost)
  : code(code),
    routes(std::move(routes)),
    summary(cost) {}
