/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/route.h"

route_t::route_t(ID_t vehicle,
                 std::vector<step>&& steps,
                 cost_t cost,
                 duration_t service,
                 const amount_t& amount)
  : vehicle(vehicle),
    steps(std::move(steps)),
    cost(cost),
    service(service),
    amount(amount),
    duration(0),
    distance(0) {
}
