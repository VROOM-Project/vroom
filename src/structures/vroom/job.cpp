/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/job.h"
#include "utils/helpers.h"

#include <numeric>

namespace vroom {

inline Duration get_tw_length(const std::vector<TimeWindow>& tws) {
  return std::accumulate(tws.begin(),
                         tws.end(),
                         0,
                         [](auto sum, const auto& tw) {
                           return sum + tw.length;
                         });
};

Job::Job(Id id,
         const Location& location,
         UserDuration setup,
         UserDuration service,
         Amount delivery,
         Amount pickup,
         Skills skills,
         Priority priority,
         const std::vector<TimeWindow>& tws,
         std::string description)
  : location(location),
    id(id),
    type(JOB_TYPE::SINGLE),
    setup(utils::scale_from_user_duration(setup)),
    service(utils::scale_from_user_duration(service)),
    delivery(std::move(delivery)),
    pickup(std::move(pickup)),
    skills(std::move(skills)),
    priority(priority),
    tws(tws),
    tw_length(get_tw_length(tws)),
    description(std::move(description)) {
  utils::check_tws(tws, id, "job");
  utils::check_priority(priority, id, "job");
}

Job::Job(Id id,
         JOB_TYPE type,
         const Location& location,
         UserDuration setup,
         UserDuration service,
         const Amount& amount,
         Skills skills,
         Priority priority,
         const std::vector<TimeWindow>& tws,
         std::string description)
  : location(location),
    id(id),
    type(type),
    setup(utils::scale_from_user_duration(setup)),
    service(utils::scale_from_user_duration(service)),
    delivery((type == JOB_TYPE::DELIVERY) ? amount : Amount(amount.size())),
    pickup((type == JOB_TYPE::PICKUP) ? amount : Amount(amount.size())),
    skills(std::move(skills)),
    priority(priority),
    tws(tws),
    tw_length(get_tw_length(tws)),
    description(std::move(description)) {
  assert(type == JOB_TYPE::PICKUP || type == JOB_TYPE::DELIVERY);
  utils::check_tws(tws, id, "job");
  utils::check_priority(priority, id, "job");
}

bool Job::is_valid_start(Duration time) const {
  bool valid = false;

  for (const auto& tw : tws) {
    if (tw.contains(time)) {
      valid = true;
      break;
    }
  }

  return valid;
}

} // namespace vroom
