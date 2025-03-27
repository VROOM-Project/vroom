/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/route.h"

namespace vroom {

Route::Route() = default;

Route::Route(Id vehicle,
             std::vector<Step>&& steps,
             UserCost cost,
             UserDuration duration,
             UserDistance distance,
             UserDuration setup,
             UserDuration service,
             UserDuration waiting_time,
             Priority priority,
             Amount delivery,
             Amount pickup,
             std::string profile,
             std::string description,
             Violations&& violations)
  : vehicle(vehicle),
    steps(std::move(steps)),
    cost(cost),
    duration(duration),
    distance(distance),
    setup(setup),
    service(service),
    waiting_time(waiting_time),
    priority(priority),
    delivery(std::move(delivery)),
    pickup(std::move(pickup)),
    profile(std::move(profile)),
    description(std::move(description)),
    violations(std::move(violations)) {
#ifndef NDEBUG
  check_timing_consistency();
#endif
}

#ifndef NDEBUG
void Route::check_timing_consistency() const {
  if (steps.empty()) {
    return;
  }

  assert((steps.front().step_type == STEP_TYPE::START &&
          steps.back().step_type == STEP_TYPE::END));

  assert(steps.back().arrival ==
         steps.front().arrival + duration + setup + service + waiting_time);

  auto current_arrival = steps.front().arrival;
  auto previous_departure = steps.front().departure();
  auto total_setup = steps.front().setup;
  auto total_service = steps.front().service;
  auto total_waiting_time = steps.front().waiting_time;

  auto previous_duration = steps.front().duration;
  assert(previous_duration == 0);

  assert(previous_departure == current_arrival);
  assert(steps.back().arrival == steps.back().departure());

  for (unsigned s = 1; s < steps.size(); ++s) {
    const auto& step = steps[s];

    current_arrival = step.arrival;
    assert(previous_departure <= current_arrival);

    auto previous_leg_duration = current_arrival - previous_departure;
    assert(step.duration == previous_duration + previous_leg_duration);

    previous_duration = step.duration;
    previous_departure = step.departure();

    total_setup += step.setup;
    total_service += step.service;
    total_waiting_time += step.waiting_time;
  }

  assert(previous_duration == duration);
  assert(total_setup == setup);
  assert(total_service == service);
  assert(total_waiting_time == waiting_time);
}
#endif

} // namespace vroom
