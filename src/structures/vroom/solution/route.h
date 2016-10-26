#ifndef ROUTE_H
#define ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./step.h"

struct route_t{
  index_t vehicle;
  std::vector<step> steps;
  duration_t cost;
  std::string geometry;
  duration_t duration;
  distance_t distance;

  route_t(index_t vehicle,
          std::vector<step> steps,
          duration_t cost):
    vehicle(vehicle),
    steps(std::move(steps)),
    cost(cost){}
};

#endif
