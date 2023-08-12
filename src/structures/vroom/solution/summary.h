#ifndef SUMMARY_H
#define SUMMARY_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/solution/computing_times.h"
#include "structures/vroom/solution/violations.h"

namespace vroom {

struct Summary {
  UserCost cost;
  const unsigned routes;
  const unsigned unassigned;
  Amount delivery;
  Amount pickup;
  UserDuration setup;
  UserDuration service;
  Priority priority;

  UserDuration duration;
  UserDuration waiting_time;
  UserDistance distance;
  ComputingTimes computing_times;

  Violations violations;

  Summary();

  Summary(unsigned routes, unsigned unassigned, const Amount& zero_amount);
};

} // namespace vroom

#endif
