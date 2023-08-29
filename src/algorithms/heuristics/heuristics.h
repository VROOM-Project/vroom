#ifndef HEURISTICS_H
#define HEURISTICS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/eval.h"
#include "structures/vroom/input/input.h"

namespace vroom::heuristics {

// Implementation of a variant of the Solomon I1 heuristic.
template <class Route, class InputIterator>
Eval basic(const Input& input,
           std::vector<Route>& routes,
           const InputIterator jobs_begin,
           const InputIterator jobs_end,
           const InputIterator vehicles_begin,
           const InputIterator vehicles_end,
           INIT init,
           double lambda = 0,
           SORT sort = SORT::CAPACITY);

// Adjusting the above for situations with heterogeneous fleet.
template <class Route, class InputIterator>
Eval dynamic_vehicle_choice(const Input& input,
                            std::vector<Route>& routes,
                            const InputIterator jobs_begin,
                            const InputIterator jobs_end,
                            const InputIterator vehicles_begin,
                            const InputIterator vehicles_end,
                            INIT init,
                            double lambda,
                            SORT sort);

// Populate routes with user-defined vehicle steps.
template <class Route>
void initial_routes(const Input& input, std::vector<Route>& routes);

} // namespace vroom::heuristics

#endif
