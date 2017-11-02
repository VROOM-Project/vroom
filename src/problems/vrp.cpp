/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "vrp.h"
#include "../structures/vroom/input/input.h"

vrp::vrp(const input& input) : _input(input) {
  assert(_input._vehicles.size() > 0);
}

vrp::~vrp() {
}

solution vrp::solve() const {
  return solve(1);
}
