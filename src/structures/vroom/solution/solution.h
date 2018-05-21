#ifndef SOLUTION_H
#define SOLUTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../job.h"
#include "./route.h"
#include "./summary.h"

struct solution {
  unsigned code;
  std::string error;
  summary_t summary;
  std::vector<route_t> routes;
  std::vector<job_t> unassigned;

  solution(unsigned code, std::string error);

  solution(unsigned code,
           cost_t cost,
           std::vector<route_t>&& routes,
           std::vector<job_t>&& unassigned,
           duration_t service,
           amount_t&& amount);
};

#endif
