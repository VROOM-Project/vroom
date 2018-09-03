/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/vroom/time_window.h"
#include "utils/exceptions.h"

constexpr duration_t time_window_t::default_length =
  std::numeric_limits<duration_t>::max();

time_window_t::time_window_t()
  : start(0), end(std::numeric_limits<duration_t>::max()), length(end - start) {
}

time_window_t::time_window_t(duration_t start, duration_t end)
  : start(start), end(end), length(end - start) {
  if (start > end) {
    throw custom_exception("Invalid time window: [" + std::to_string(start) +
                           ", " + std::to_string(end) + "]");
  }
}

bool time_window_t::contains(duration_t time) const {
  return (start <= time) and (time <= end);
}

bool time_window_t::is_default() const {
  return end - start == default_length;
}

bool operator<(const time_window_t& lhs, const time_window_t& rhs) {
  return lhs.start < rhs.start or
         (lhs.start == rhs.start and lhs.end < rhs.end);
}
