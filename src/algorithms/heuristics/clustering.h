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

class Clustering {
private:
  const Input& input_ref;
  void parallel_clustering();
  void sequential_clustering();

public:
  const CLUSTERING type;
  const INIT init;
  const float regret_coeff;
  // Clusters are relative to the vehicle with same rank in
  // input_ref._vehicles.
  std::vector<std::vector<Index>> clusters;
  // Cost of all edges added during the clustering process
  Cost edges_cost;
  unsigned assigned_jobs;
  unsigned non_empty_clusters;

  Clustering(const Input& input, CLUSTERING t, INIT i, float c);
};

#endif
