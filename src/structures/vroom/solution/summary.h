#ifndef SUMMARY_H
#define SUMMARY_H

/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/amount.h"
#include "structures/vroom/solution/computing_times.h"

namespace vroom {

struct Summary {
  Cost cost;
  const unsigned unassigned;
  Amount amount;
  Duration service;

  Duration duration;
  Duration waiting_time;
  Distance distance;
  ComputingTimes computing_times;

  Summary();

  Summary(unsigned unassigned, unsigned amount_size);
};

} // namespace vroom

#endif
