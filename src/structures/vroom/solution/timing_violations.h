#ifndef TIMING_VIOLATIONS_H
#define TIMING_VIOLATIONS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom {

struct TimingViolations {
 Duration start_lead_time;
 Duration lead_time;
 Duration end_delay;
 Duration delay;

  TimingViolations();

  TimingViolations(const Duration start_lead_time,
                   const Duration lead_time,
                   const Duration end_delay,
                   const Duration delay);

  TimingViolations& operator+=(const TimingViolations& rhs);
};

} // namespace vroom

#endif
