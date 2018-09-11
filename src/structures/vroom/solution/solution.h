#ifndef SOLUTION_H
#define SOLUTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/vroom/job.h"
#include "structures/vroom/solution/route.h"
#include "structures/vroom/solution/summary.h"

struct solution {
  unsigned code;
  std::string error;
  summary_t summary;
  std::vector<route_t> routes;
  std::vector<job_t> unassigned;

  solution(unsigned code, std::string error);

  solution(unsigned code,
           unsigned amount_size,
           std::vector<route_t>&& routes,
           std::vector<job_t>&& unassigned);
};

#endif
