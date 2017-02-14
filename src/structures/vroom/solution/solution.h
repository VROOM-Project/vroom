#ifndef SOLUTION_H
#define SOLUTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./route.h"
#include "./summary.h"

struct solution{
  index_t code;
  std::string error;
  std::vector<route_t> routes;
  summary_t summary;

  solution(index_t code, std::string error);

  solution(index_t code,
           std::vector<route_t>&& routes,
           duration_t cost);
};

#endif
