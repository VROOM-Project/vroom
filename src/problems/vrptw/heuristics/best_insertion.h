#ifndef BEST_INSERTION_H
#define BEST_INSERTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

tw_solution best_insertion(const input& input);

#endif
