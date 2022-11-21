/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>

#include "structures/vroom/input/vehicle_step.h"

namespace vroom {

ForcedService::ForcedService()
  : at(std::optional<Duration>()),
    after(std::optional<Duration>()),
    before(std::optional<Duration>()) {
}

ForcedService::ForcedService(const std::optional<UserDuration>& at,
                             const std::optional<UserDuration>& after,
                             const std::optional<UserDuration>& before) {
  if (at.has_value()) {
    this->at = DURATION_FACTOR * at.value();
  }
  if (after.has_value()) {
    this->after = DURATION_FACTOR * after.value();
  }
  if (before.has_value()) {
    this->before = DURATION_FACTOR * before.value();
  }
}

VehicleStep::VehicleStep(STEP_TYPE type, ForcedService&& forced_service)
  : id(0),
    type(type),
    job_type(JOB_TYPE::SINGLE),
    forced_service(std::move(forced_service)) {
  assert(type == STEP_TYPE::START or type == STEP_TYPE::END);
}

VehicleStep::VehicleStep(STEP_TYPE type, Id id, ForcedService&& forced_service)
  : id(id),
    type(type),
    job_type(JOB_TYPE::SINGLE),
    forced_service(std::move(forced_service)) {
  assert(type == STEP_TYPE::BREAK);
}

VehicleStep::VehicleStep(JOB_TYPE job_type,
                         Id id,
                         ForcedService&& forced_service)
  : id(id),
    type(STEP_TYPE::JOB),
    job_type(job_type),
    forced_service(std::move(forced_service)) {
}

} // namespace vroom
