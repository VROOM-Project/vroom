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

// Implementation of a variant of the Solomon I1 heuristic.
tw_solution solomon(const input& input, float lambda);

#endif
