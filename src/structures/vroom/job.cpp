/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/job.h"
#include "utils/helpers.h"

namespace vroom {

Job::Job(Id id,
         const Location& location,
         UserDuration default_setup,
         UserDuration default_service,
         Amount delivery,
         Amount pickup,
         Skills skills,
         Priority priority,
         const std::vector<TimeWindow>& tws,
         std::string description,
         const TypeToUserDurationMap& setup_per_type,
         const TypeToUserDurationMap& service_per_type)
  : location(location),
    id(id),
    type(JOB_TYPE::SINGLE),
    default_setup(utils::scale_from_user_duration(default_setup)),
    default_service(utils::scale_from_user_duration(default_service)),
    delivery(std::move(delivery)),
    pickup(std::move(pickup)),
    skills(std::move(skills)),
    priority(priority),
    tws(tws),
    description(std::move(description)),
    setup_per_type(utils::scale_from_user_duration(setup_per_type)),
    service_per_type(utils::scale_from_user_duration(service_per_type)) {
  utils::check_tws(tws, id, "job");
  utils::check_priority(priority, id, "job");
  utils::check_no_empty_keys(this->setup_per_type, id, "job", "setup_per_type");
  utils::check_no_empty_keys(this->service_per_type,
                             id,
                             "job",
                             "service_per_type");
}

Job::Job(Id id,
         JOB_TYPE type,
         const Location& location,
         UserDuration default_setup,
         UserDuration default_service,
         const Amount& amount,
         Skills skills,
         Priority priority,
         const std::vector<TimeWindow>& tws,
         std::string description,
         const TypeToUserDurationMap& setup_per_type,
         const TypeToUserDurationMap& service_per_type)
  : location(location),
    id(id),
    type(type),
    default_setup(utils::scale_from_user_duration(default_setup)),
    default_service(utils::scale_from_user_duration(default_service)),
    delivery((type == JOB_TYPE::DELIVERY) ? amount : Amount(amount.size())),
    pickup((type == JOB_TYPE::PICKUP) ? amount : Amount(amount.size())),
    skills(std::move(skills)),
    priority(priority),
    tws(tws),
    description(std::move(description)),
    setup_per_type(utils::scale_from_user_duration(setup_per_type)),
    service_per_type(utils::scale_from_user_duration(service_per_type)) {
  assert(type == JOB_TYPE::PICKUP || type == JOB_TYPE::DELIVERY);
  std::string type_str = (type == JOB_TYPE::PICKUP) ? "pickup" : "delivery";
  utils::check_tws(tws, id, type_str);
  utils::check_priority(priority, id, type_str);
  utils::check_no_empty_keys(this->setup_per_type,
                             id,
                             type_str,
                             "setup_per_type");
  utils::check_no_empty_keys(this->service_per_type,
                             id,
                             type_str,
                             "service_per_type");
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
