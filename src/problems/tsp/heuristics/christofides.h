#ifndef CHRISTOFIDES_H
#define CHRISTOFIDES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <list>
#include "structures/typedefs.h"
#include "structures/abstract/matrix.h"

// Implementing a variant of the Christofides heuristic.
std::list<index_t> christofides(const matrix<cost_t>& sym_matrix);

#endif
