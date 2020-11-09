/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/timing_violations.h"

namespace vroom {

TimingViolations::TimingViolations()
  : start_lead_time(0), lead_time(0), end_delay(0), delay(0) {
}

TimingViolations::TimingViolations(const Duration start_lead_time,
                                   const Duration lead_time,
                                   const Duration end_delay,
                                   const Duration delay)
  : start_lead_time(start_lead_time),
    lead_time(lead_time),
    end_delay(end_delay),
    delay(delay) {
}

TimingViolations& TimingViolations::operator+=(const TimingViolations& rhs) {
  this->start_lead_time += rhs.start_lead_time;
  this->lead_time += rhs.lead_time;
  this->end_delay += rhs.end_delay;
  this->delay += rhs.delay;

  return *this;
}

} // namespace vroom
