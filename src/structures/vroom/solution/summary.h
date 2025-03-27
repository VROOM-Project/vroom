#ifndef SUMMARY_H
#define SUMMARY_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/solution/computing_times.h"
#include "structures/vroom/solution/violations.h"

namespace vroom {

struct Summary {
  UserCost cost{0};
  const unsigned routes;
  const unsigned unassigned;
  Amount delivery;
  Amount pickup;
  UserDuration setup{0};
  UserDuration service{0};
  Priority priority{0};

  UserDuration duration{0};
  UserDuration waiting_time{0};
  UserDistance distance{0};
  ComputingTimes computing_times;

  Violations violations{0, 0};

  Summary();

  Summary(unsigned routes, unsigned unassigned, const Amount& zero_amount);
};

} // namespace vroom

#endif
