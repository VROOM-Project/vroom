#ifndef ROUTE_H
#define ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./step.h"

struct route{
  const index_t vehicle;
  const std::vector<step> steps;
  const duration_t cost;
  const boost::optional<std::string> geometry;
  const boost::optional<duration_t> duration;
  const boost::optional<distance_t> distance;

  route(const index_t vehicle,
        const std::vector<step>& steps,
        const duration_t cost):
    vehicle(vehicle),
    steps(std::move(steps)),
    cost(cost){}

  route(const index_t vehicle,
        const std::vector<step>& steps,
        const duration_t cost,
        const boost::optional<std::string>& geometry,
        const duration_t duration,
        const distance_t distance):
    route(vehicle, steps, cost),
    geometry(std::move(geometry)),
    duration(duration),
    distance(distance){}
};

#endif
