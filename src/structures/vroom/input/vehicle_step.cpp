/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>

#include "structures/vroom/input/vehicle_step.h"

namespace vroom {

ForcedService::ForcedService(const std::optional<UserDuration>& at,
                             const std::optional<UserDuration>& after,
                             const std::optional<UserDuration>& before) {
  if (at.has_value()) {
    this->at = utils::scale_from_user_duration(at.value());
  }
  if (after.has_value()) {
    this->after = utils::scale_from_user_duration(after.value());
  }
  if (before.has_value()) {
    this->before = utils::scale_from_user_duration(before.value());
  }
}

VehicleStep::VehicleStep(STEP_TYPE type, ForcedService&& forced_service)
  : id(0), type(type), forced_service(std::move(forced_service)) {
  assert(type == STEP_TYPE::START || type == STEP_TYPE::END);
}

VehicleStep::VehicleStep(STEP_TYPE type, Id id, ForcedService&& forced_service)
  : id(id), type(type), forced_service(std::move(forced_service)) {
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
