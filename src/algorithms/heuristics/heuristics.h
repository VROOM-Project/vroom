#ifndef HEURISTICS_H
#define HEURISTICS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_set>

#include "structures/vroom/eval.h"
#include "structures/vroom/input/input.h"

namespace vroom::heuristics {

// Implementation of a variant of the Solomon I1 heuristic.
template <class Route, std::forward_iterator Iter>
Eval basic(const Input& input,
           std::vector<Route>& routes,
           const Iter jobs_begin,
           const Iter jobs_end,
           const Iter vehicles_begin,
           const Iter vehicles_end,
           INIT init,
           double lambda,
           SORT sort);

// Adjusting the above for situations with heterogeneous fleet.
template <class Route, std::forward_iterator Iter>
Eval dynamic_vehicle_choice(const Input& input,
                            std::vector<Route>& routes,
                            const Iter jobs_begin,
                            const Iter jobs_end,
                            const Iter vehicles_begin,
                            const Iter vehicles_end,
                            INIT init,
                            double lambda,
                            SORT sort);

// Populate routes with user-defined vehicle steps.
template <class Route>
std::unordered_set<Index> set_initial_routes(const Input& input,
                                             std::vector<Route>& routes);

} // namespace vroom::heuristics

#endif
