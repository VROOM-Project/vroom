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
template <class Route>
Eval basic(const Input& input,
           std::vector<Route>& routes,
           INIT init,
           double lambda,
           SORT sort);

// Adjusting the above for situation with heterogeneous fleet.
template <class Route>
Eval dynamic_vehicle_choice(const Input& input,
                            std::vector<Route>& routes,
                            INIT init,
                            double lambda,
                            SORT sort);

// Populate routes with user-defined vehicle steps.
template <class Route>
void initial_routes(const Input& input, std::vector<Route>& routes);

} // namespace vroom::heuristics

#endif
