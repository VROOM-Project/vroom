/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/vroom/time_window.h"
#include "utils/exceptions.h"

time_window_t::time_window_t(duration_t start, duration_t end)
  : start(start), end(end) {
  if (start > end) {
    throw custom_exception("Invalid time window: [" + std::to_string(start) +
                           ", " + std::to_string(end) + "]");
  }
}
