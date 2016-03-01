#ifndef CHRISTO_HEURISTIC_H
#define CHRISTO_HEURISTIC_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>
#include <unordered_map>
#include <set>
#include <algorithm>
#include <chrono>
#include <random>
#include <boost/log/trivial.hpp>
#include "heuristic.h"
#include "../algorithms/kruskal.h"
#include "../algorithms/munkres.h"
#include "../structures/tsp.h"
#include "../structures/edge.h"
#include "../structures/undirected_graph.h"
#include "../structures/matrix.h"

// Implementing the Christofides heuristic.
class christo_heuristic : public heuristic{
public:

  virtual std::list<index_t> build_solution(const tsp& instance) override;
};

#endif
