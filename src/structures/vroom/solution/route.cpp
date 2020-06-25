/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
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
             const Amount& delivery,
             const Amount& pickup,
             const std::string& description)
  : vehicle(vehicle),
    steps(std::move(steps)),
    cost(cost),
    service(service),
    duration(duration),
    waiting_time(waiting_time),
    delivery(delivery),
    pickup(pickup),
    description(description),
    distance(0) {
}

} // namespace vroom
