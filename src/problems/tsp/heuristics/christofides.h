#ifndef CHRISTOFIDES_H
#define CHRISTOFIDES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <list>

#include "utils/helpers.h"

namespace vroom::tsp {

// Implementing a variant of the Christofides heuristic.
std::list<Index> christofides(const Matrix<UserCost>& sym_matrix);

} // namespace vroom::tsp

#endif
