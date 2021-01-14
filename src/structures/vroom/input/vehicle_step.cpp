/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
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

ForcedService::ForcedService(std::optional<Duration>&& at,
                             std::optional<Duration>&& after,
                             std::optional<Duration>&& before)
  : at(std::move(at)), after(std::move(after)), before(std::move(before)) {
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
