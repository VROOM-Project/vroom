#ifndef BREAK_H
#define BREAK_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/time_window.h"

namespace vroom {

struct Break {
  const Id id;
  const Duration service;
  const std::vector<TimeWindow> tws;

  // Constructor for regular one-stop job (JOB_TYPE::SINGLE).
  Break(Id id, const std::vector<TimeWindow>& tws, Duration service = 0);
};

} // namespace vroom

#endif
