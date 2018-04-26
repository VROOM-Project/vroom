/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "step.h"

// Dummy initialization value for unused job id.
step::step(TYPE type, location_t location)
  : type(type), location(location), job(0) {
  assert(type == TYPE::START or type == TYPE::END);
}

step::step(TYPE type, location_t location, ID_t job)
  : type(type), location(location), job(job) {
}
