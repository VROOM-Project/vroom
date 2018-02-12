#ifndef CLUSTERING_H
#define CLUSTERING_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <unordered_set>
#include <vector>

#include "../../../structures/vroom/amount.h"
#include "../../../structures/vroom/input/input.h"
#include "../../../structures/vroom/job.h"

struct clustering {
  std::string strategy;
  double regret_coeff;
  std::vector<std::vector<index_t>> clusters;
  // Cost of all edges added during the clustering process
  cost_t edges_cost;
  std::unordered_set<job_t> unassigned;

  clustering(std::string s, double c, std::size_t V);
};

clustering parallel_clustering(const input& input, double regret_coeff);

clustering sequential_clustering(const input& input, double regret_coeff);

#endif
