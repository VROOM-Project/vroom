#ifndef JOB_H
#define JOB_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/location.h"
#include "structures/vroom/time_window.h"

namespace vroom {

struct Job {
  const Id id;
  Location location;
  const Duration service;
  const Amount amount;
  const Skills skills;
  const std::vector<TimeWindow> tws;
  const Duration tw_length;

  Job(Id id,
      const Location& location,
      Duration service = 0,
      const Amount& amount = Amount(0),
      const Skills& skills = Skills(),
      const std::vector<TimeWindow>& tws =
        std::vector<TimeWindow>(1, TimeWindow()));

  Index index() const;

  bool is_valid_start(Duration time) const;
};

} // namespace vroom

#endif
