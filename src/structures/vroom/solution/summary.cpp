/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "summary.h"

summary_t::summary_t(cost_t cost, unsigned unassigned)
  : cost(cost), service(0), duration(0), distance(0), unassigned(unassigned) {
}
