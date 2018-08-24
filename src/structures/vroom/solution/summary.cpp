/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/summary.h"

summary_t::summary_t() : cost(0), unassigned(0), service(0), amount() {
}

summary_t::summary_t(cost_t cost,
                     unsigned unassigned,
                     duration_t service,
                     amount_t&& amount)
  : cost(cost),
    unassigned(unassigned),
    service(service),
    amount(std::move(amount)),
    duration(0),
    waiting_time(0),
    distance(0) {
}
