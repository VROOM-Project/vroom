/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>

#include "structures/vroom/input/input_step.h"

namespace vroom {

InputStep::InputStep(STEP_TYPE type)
  : id(0), type(type), job_type(JOB_TYPE::SINGLE) {
  assert(type == STEP_TYPE::START or type == STEP_TYPE::END);
}

InputStep::InputStep(STEP_TYPE type, Id id)
  : id(id), type(type), job_type(JOB_TYPE::SINGLE) {
  assert(type == STEP_TYPE::BREAK);
}

InputStep::InputStep(JOB_TYPE job_type, Id id)
  : id(id), type(STEP_TYPE::JOB), job_type(job_type) {
}

} // namespace vroom
