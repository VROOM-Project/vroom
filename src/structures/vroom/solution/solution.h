#ifndef SOLUTION_H
#define SOLUTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./route.h"
#include "./summary.h"

struct solution {
  unsigned code;
  std::string error;
  std::vector<route_t> routes;
  summary_t summary;

  solution(unsigned code, std::string error);

  solution(unsigned code, std::vector<route_t>&& routes, cost_t cost);
};

#endif
