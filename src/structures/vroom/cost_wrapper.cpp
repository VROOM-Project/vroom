/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/cost_wrapper.h"

namespace vroom {

CostWrapper::CostWrapper(double speed_factor)
  : durations_factor(1. / speed_factor) {
}

void CostWrapper::set_durations_matrix(const Matrix<Cost>* matrix) {
  durations_matrix = matrix;
}

} // namespace vroom
