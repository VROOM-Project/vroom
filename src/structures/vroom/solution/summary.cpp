/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/summary.h"

namespace vroom {

Summary::Summary() : cost(0), routes(0), unassigned(0), setup(0), service(0) {
}

Summary::Summary(unsigned routes,
                 unsigned unassigned,
                 const Amount& zero_amount)
  : routes(routes),
    unassigned(unassigned),
    delivery(zero_amount),
    pickup(zero_amount) {
}

} // namespace vroom
