#ifndef SOLUTION_H
#define SOLUTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <vector>

#include "structures/vroom/job.h"
#include "structures/vroom/solution/route.h"
#include "structures/vroom/solution/summary.h"

namespace vroom {

struct Solution {
  Summary summary;
  std::vector<Route> routes;
  std::vector<Job> unassigned;

  Solution(const Amount& zero_amount,
           std::vector<Route>&& routes,
           std::vector<Job>&& unassigned);
};

} // namespace vroom

#endif
