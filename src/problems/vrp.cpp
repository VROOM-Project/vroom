/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrp.h"
#include "structures/vroom/input/input.h"

namespace vroom {

VRP::VRP(const Input& input) : _input(input) {
  assert(!_input.vehicles.empty());
}

VRP::~VRP() = default;

} // namespace vroom
