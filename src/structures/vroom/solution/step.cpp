/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/step.h"
#include "utils/helpers.h"

namespace vroom {

// Used for start and end steps.
Step::Step(STEP_TYPE type, Location location, const Amount& load)
  : step_type(type),
    job_type(JOB_TYPE::SINGLE), // Dummy init.
    location(location),
    id(0),
    setup(0),
    service(0),
    load(load),
    arrival(0),
    duration(0),
    waiting_time(0) {
  assert(step_type == STEP_TYPE::START or step_type == STEP_TYPE::END);
}

Step::Step(const Job& job, const UserDuration setup, const Amount& load)
  : step_type(STEP_TYPE::JOB),
    job_type(job.type),
    location(job.location),
    id(job.id),
    setup(setup),
    service(utils::scale_to_user_duration(job.service)),
    load(load),
    description(job.description),
    waiting_time(0) {
}

Step::Step(const Break& b, const Amount& load)
  : step_type(STEP_TYPE::BREAK),
    job_type(JOB_TYPE::SINGLE), // Dummy value.
    location(0),                // Dummy value.
    id(b.id),
    setup(0),
    service(utils::scale_to_user_duration(b.service)),
    load(load),
    description(b.description),
    waiting_time(0) {
}

UserDuration Step::departure() const {
  return arrival + waiting_time + setup + service;
}

} // namespace vroom
