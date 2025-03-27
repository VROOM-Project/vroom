#ifndef HEURISTICS_H
#define HEURISTICS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <set>
#include <unordered_set>

#include "structures/vroom/eval.h"
#include "structures/vroom/input/input.h"

namespace vroom::heuristics {

// Implementation of a variant of the Solomon I1 heuristic.
template <class Route>
Eval basic(const Input& input,
           std::vector<Route>& routes,
           std::set<Index> unassigned,
           std::vector<Index> vehicles_ranks,
           INIT init,
           double lambda,
           SORT sort);

// Adjusting the above for situations with heterogeneous fleet.
template <class Route>
Eval dynamic_vehicle_choice(const Input& input,
                            std::vector<Route>& routes,
                            std::set<Index> unassigned,
                            std::vector<Index> vehicles_ranks,
                            INIT init,
                            double lambda,
                            SORT sort);

// Populate routes with user-defined vehicle steps.
template <class Route>
void set_initial_routes(const Input& input,
                        std::vector<Route>& routes,
                        std::unordered_set<Index>& assigned);

} // namespace vroom::heuristics

#endif
