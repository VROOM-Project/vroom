#ifndef MST_HEURISTIC_H
#define MST_HEURISTIC_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <stack>
#include <boost/log/trivial.hpp>
#include "heuristic.h"
#include "../algorithms/kruskal.h"
#include "../structures/typedefs.h"
#include "../structures/tsp.h"
#include "../structures/edge.h"
#include "../structures/undirected_graph.h"

// Simple heuristic based on a deep-search on a minimal spanning tree.
class mst_heuristic : public heuristic{
public:

  virtual std::list<index_t> build_solution(const tsp& instance) override;
};

#endif
