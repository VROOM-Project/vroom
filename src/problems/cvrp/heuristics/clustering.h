#ifndef CLUSTERING_H
#define CLUSTERING_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/vroom/input/input.h"
#include "structures/vroom/job.h"

// Clustering types.
enum class CLUSTERING_T { PARALLEL, SEQUENTIAL };

// Initialization types.
enum class INIT_T { NONE, HIGHER_AMOUNT, NEAREST };

class clustering {
private:
  const input& input_ref;
  void parallel_clustering();
  void sequential_clustering();

public:
  const CLUSTERING_T type;
  const INIT_T init;
  const double regret_coeff;
  // Clusters are relative to the vehicle with same rank in
  // input_ref._vehicles.
  std::vector<std::vector<index_t>> clusters;
  // Cost of all edges added during the clustering process
  cost_t edges_cost;
  unsigned assigned_jobs;
  unsigned non_empty_clusters;

  clustering(const input& input, CLUSTERING_T t, INIT_T i, double c);

  friend bool operator<(const clustering& lhs, const clustering& rhs);
};

#endif
