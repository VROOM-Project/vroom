/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/summary.h"

namespace vroom {

Summary::Summary() : cost(0), unassigned(0), amount(), service(0) {
}

Summary::Summary(unsigned unassigned, unsigned amount_size)
  : cost(0),
    unassigned(unassigned),
    amount(amount_size),
    service(0),
    duration(0),
    waiting_time(0),
    distance(0) {
}

} // namespace vroom
