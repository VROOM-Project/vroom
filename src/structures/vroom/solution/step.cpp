/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/step.h"
#include "utils/helpers.h"

namespace vroom {

// Used for start and end steps.
Step::Step(STEP_TYPE type, Location location, Amount load)
  : step_type(type),
    location(location),
    id(0),
    service(0),
    load(std::move(load)) {
  assert(step_type == STEP_TYPE::START || step_type == STEP_TYPE::END);
}

Step::Step(const Job& job,
           const UserDuration setup,
           const UserDuration service,
           Amount load)
  : step_type(STEP_TYPE::JOB),
    job_type(job.type),
    location(job.location),
    id(job.id),
    setup(setup),
    service(service),
    load(std::move(load)),
    description(job.description) {
}

Step::Step(const Break& b, Amount load)
  : step_type(STEP_TYPE::BREAK),
    id(b.id),
    service(utils::scale_to_user_duration(b.service)),
    load(std::move(load)),
    description(b.description) {
}

UserDuration Step::departure() const {
  return arrival + waiting_time + setup + service;
}

void Step::update_cargo_lifetime(const Job& job, UserDuration current_time) {
  if (job.has_lifetime_constraint()) {
    if (job.type == JOB_TYPE::PICKUP) {
      cargo_pickup_time = current_time;
    } else if (job.type == JOB_TYPE::DELIVERY) {
      cargo_age = current_time - utils::scale_to_user_duration(job.pickup_time);
      max_cargo_lifetime = utils::scale_to_user_duration(job.max_lifetime);
      has_expired_cargo = cargo_age > max_cargo_lifetime;
    }
  }
}

} // namespace vroom
