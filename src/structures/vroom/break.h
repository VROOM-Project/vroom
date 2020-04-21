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
  Id id;
  std::vector<TimeWindow> tws;
  Duration service;

  Break(Id id, const std::vector<TimeWindow>& tws, Duration service = 0);

  bool is_valid_start(Duration time) const;
};

} // namespace vroom

#endif
