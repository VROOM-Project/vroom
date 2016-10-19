#ifndef SOLUTION_H
#define SOLUTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./computing_times.h"

struct solution_t{
  const duration_t cost;
  const boost::optional<duration_t> duration;
  const boost::optional<distance_t> distance;
  const computing_times_t computing_times;

  solution_t(duration_t cost,
             computing_times_t computing_times):
    cost(cost),
    duration(boost::none),
    distance(boost::none),
    computing_times(computing_times){}
};

#endif
