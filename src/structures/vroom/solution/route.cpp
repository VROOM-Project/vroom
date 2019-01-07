/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/route.h"

namespace vroom {

Route::Route(Id vehicle,
             std::vector<Step>&& steps,
             Cost cost,
             Duration service,
             Duration duration,
             Duration waiting_time,
             const Amount& amount)
  : vehicle(vehicle),
    steps(std::move(steps)),
    cost(cost),
    service(service),
    duration(duration),
    waiting_time(waiting_time),
    amount(amount),
    distance(0) {
}

} // namespace vroom
