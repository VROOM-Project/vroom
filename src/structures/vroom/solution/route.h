#ifndef ROUTE_H
#define ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <vector>

#include "structures/vroom/solution/step.h"

struct route_t {
  const ID_t vehicle;
  std::vector<step> steps;
  const cost_t cost;
  const duration_t service;
  const duration_t duration;
  const duration_t waiting_time;
  const amount_t amount;

  std::string geometry;
  distance_t distance;

  route_t(ID_t vehicle,
          std::vector<step>&& steps,
          cost_t cost,
          duration_t service,
          duration_t duration,
          duration_t waiting_time,
          const amount_t& amount);
};

#endif
