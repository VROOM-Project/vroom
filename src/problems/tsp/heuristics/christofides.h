#ifndef CHRISTOFIDES_H
#define CHRISTOFIDES_H

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
#include "../../../algorithms/kruskal.h"
#include "../../../algorithms/munkres.h"
#include "../../../structures/abstract/edge.h"
#include "../../../structures/abstract/undirected_graph.h"
#include "../../../structures/abstract/matrix.h"

// Implementing a variant of the Christofides heuristic.
std::list<index_t> christofides(const matrix<distance_t>& sym_matrix);

#endif
