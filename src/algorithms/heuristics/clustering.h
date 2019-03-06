#ifndef CLUSTERING_H
#define CLUSTERING_H

/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/vroom/input/input.h"
#include "structures/vroom/job.h"

namespace vroom {
namespace heuristics {

class Clustering {
private:
  const Input& _input;
  const CLUSTERING _type;
  const INIT _init;
  const float _regret_coeff;

  void parallel_clustering();
  void sequential_clustering();

public:
  // Clusters are relative to the vehicle with same rank in
  // _input.vehicles.
  std::vector<std::vector<Index>> clusters;
  // Cost of all edges added during the clustering process
  Cost edges_cost;
  unsigned assigned_jobs;
  unsigned non_empty_clusters;

  Clustering(const Input& input, CLUSTERING t, INIT i, float c);
};

} // namespace heuristics
} // namespace vroom

#endif
