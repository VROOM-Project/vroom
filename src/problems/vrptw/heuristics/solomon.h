#ifndef SOLOMON_H
#define SOLOMON_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

// Initialization types.
enum class INIT_T { NONE, HIGHER_AMOUNT, EARLIEST_DEADLINE, FURTHEST };

// Implementation of a variant of the Solomon I1 heuristic.
tw_solution homogeneous_solomon(const input& input, INIT_T init, float lambda);

#endif
