/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./route.h"

route_t::route_t(index_t vehicle,
                 std::vector<step> steps,
                 duration_t cost):
  vehicle(vehicle),
  steps(std::move(steps)),
  cost(cost),
  duration(0),
  distance(0){}

