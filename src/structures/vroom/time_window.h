#ifndef TIME_WINDOW_H
#define TIME_WINDOW_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom {

struct TimeWindow {
  static const Duration default_length;
  Duration start;
  Duration end;
  Duration length;

  // Default "no-constraint" time-window.
  TimeWindow();

  TimeWindow(UserDuration start, UserDuration end);

  bool contains(Duration time) const {
    return (start <= time) && (time <= end);
  }

  bool is_default() const {
    return end - start == default_length;
  }

  friend bool operator<(const TimeWindow& lhs, const TimeWindow& rhs) {
    return lhs.start < rhs.start ||
           (lhs.start == rhs.start && lhs.end < rhs.end);
  }
};

} // namespace vroom

#endif
