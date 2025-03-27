/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/time_window.h"
#include "utils/exception.h"

namespace vroom {

constexpr Duration TimeWindow::default_length =
  utils::scale_from_user_duration(std::numeric_limits<UserDuration>::max());

TimeWindow::TimeWindow()
  : start(0),
    end(utils::scale_from_user_duration(
      std::numeric_limits<UserDuration>::max())),
    length(end - start) {
}

TimeWindow::TimeWindow(UserDuration start, UserDuration end)
  : start(utils::scale_from_user_duration(start)),
    end(utils::scale_from_user_duration(end)),
    length(utils::scale_from_user_duration(end - start)) {
  if (start > end) {
    throw InputException(
      std::format("Invalid time window: [{}, {}]", start, end));
  }
}

bool TimeWindow::contains(Duration time) const {
  return (start <= time) && (time <= end);
}

bool TimeWindow::is_default() const {
  return end - start == default_length;
}

bool operator<(const TimeWindow& lhs, const TimeWindow& rhs) {
  return lhs.start < rhs.start || (lhs.start == rhs.start && lhs.end < rhs.end);
}

} // namespace vroom
