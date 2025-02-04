#ifndef JOB_H
#define JOB_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/location.h"
#include "structures/vroom/time_window.h"

namespace vroom {

struct Job {
  Location location;
  const Id id;
  const JOB_TYPE type;
  const Duration setup;
  const Duration base_service;
  std::vector<Duration> vehicle_service;
  const Amount delivery;
  const Amount pickup;
  const Skills skills;
  const Priority priority;
  const std::vector<TimeWindow> tws;
  const std::string description;

  // Constructor for regular one-stop job (JOB_TYPE::SINGLE).
  Job(Id id,
      const Location& location,
      UserDuration setup = 0,
      UserDuration service = 0,
      Amount delivery = Amount(0),
      Amount pickup = Amount(0),
      Skills skills = Skills(),
      Priority priority = 0,
      const std::vector<TimeWindow>& tws =
        std::vector<TimeWindow>(1, TimeWindow()),
      std::string description = "");

  // Constructor for pickup and delivery jobs (JOB_TYPE::PICKUP or
  // JOB_TYPE::DELIVERY).
  Job(Id id,
      JOB_TYPE type,
      const Location& location,
      UserDuration setup = 0,
      UserDuration service = 0,
      const Amount& amount = Amount(0),
      Skills skills = Skills(),
      Priority priority = 0,
      const std::vector<TimeWindow>& tws =
        std::vector<TimeWindow>(1, TimeWindow()),
      std::string description = "");

  Index index() const {
    return location.index();
  }

  bool is_valid_start(Duration time) const;
};

} // namespace vroom

#endif
