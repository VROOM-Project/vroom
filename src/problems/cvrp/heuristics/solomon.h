#ifndef CVRP_SOLOMON_H
#define CVRP_SOLOMON_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"
#include "structures/vroom/raw_route.h"

using raw_solution = std::vector<raw_route>;

// Implementation of a variant of the Solomon I1 heuristic.
raw_solution cvrp_basic_heuristic(const input& input,
                                  INIT_T init,
                                  float lambda);

// Adjusting the above for situation with heterogeneous fleet.
raw_solution cvrp_dynamic_vehicle_choice_heuristic(const input& input,
                                                   INIT_T init,
                                                   float lambda);

#endif
