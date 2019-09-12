/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/step.h"

namespace vroom {

// Dummy initialization value for unused job id.
Step::Step(STEP_TYPE type, Location location, const Amount& load)
  : type(type),
    location(location),
    job(0),
    service(0),
    load(load),
    waiting_time(0) {
  assert(type == STEP_TYPE::START or type == STEP_TYPE::END);
}

Step::Step(const Job& job, const Amount& load)
  : type(STEP_TYPE::JOB),
    location(job.location),
    job(job.id),
    service(job.service),
    load(load),
    waiting_time(0) {
}

} // namespace vroom
