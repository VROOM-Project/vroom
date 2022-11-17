/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/route.h"

namespace vroom {

Route::Route() {
}

Route::Route(Id vehicle,
             std::vector<Step>&& steps,
             Cost cost,
             Duration setup,
             Duration service,
             Duration duration,
             Duration waiting_time,
             Priority priority,
             const Amount& delivery,
             const Amount& pickup,
             const std::string& profile,
             const std::string& description,
             const Violations&& violations)
  : vehicle(vehicle),
    steps(std::move(steps)),
    cost(cost / DURATION_FACTOR),
    setup(setup / DURATION_FACTOR),
    service(service / DURATION_FACTOR),
    duration(duration / DURATION_FACTOR),
    waiting_time(waiting_time / DURATION_FACTOR),
    priority(priority),
    delivery(delivery),
    pickup(pickup),
    profile(profile),
    description(description),
    violations(std::move(violations)),
    distance(0) {
  assert(steps.empty() or (steps.front().step_type == STEP_TYPE::START and
                           steps.back().step_type == STEP_TYPE::END));

  // Scale values back to seconds.
  this->violations.lead_time /= DURATION_FACTOR;
  this->violations.delay /= DURATION_FACTOR;

  for (auto& step : this->steps) {
    step.setup /= DURATION_FACTOR;
    step.service /= DURATION_FACTOR;
    step.arrival /= DURATION_FACTOR;
    step.duration /= DURATION_FACTOR;
    step.waiting_time /= DURATION_FACTOR;

    step.violations.lead_time /= DURATION_FACTOR;
    step.violations.delay /= DURATION_FACTOR;
  }
}

} // namespace vroom
