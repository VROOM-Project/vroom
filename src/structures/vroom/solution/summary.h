#ifndef SUMMARY_H
#define SUMMARY_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/amount.h"
#include "structures/vroom/solution/computing_times.h"
#include "structures/vroom/solution/violations.h"

namespace vroom {

struct Summary {
  Cost cost;
  const unsigned routes;
  const unsigned unassigned;
  Amount delivery;
  Amount pickup;
  Duration setup;
  Duration service;
  Priority priority;

  Duration duration;
  Duration waiting_time;
  Distance distance;
  ComputingTimes computing_times;

  Violations violations;

  Summary();

  Summary(unsigned routes, unsigned unassigned, unsigned amount_size);
};

} // namespace vroom

#endif
