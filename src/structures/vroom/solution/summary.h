#ifndef SUMMARY_H
#define SUMMARY_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/amount.h"
#include "structures/vroom/solution/computing_times.h"

struct summary_t {
  const cost_t cost;
  const unsigned unassigned;
  const duration_t service;
  const amount_t amount;

  duration_t duration;
  duration_t waiting_time;
  distance_t distance;
  computing_times_t computing_times;

  summary_t();

  summary_t(cost_t cost,
            unsigned unassigned,
            duration_t service,
            amount_t&& amount);
};

#endif
