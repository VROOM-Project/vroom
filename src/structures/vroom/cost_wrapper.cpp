/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/cost_wrapper.h"
#include <cmath>

namespace vroom {

CostWrapper::CostWrapper(double speed_factor)
  : discrete_factor(std::round(1 / speed_factor * DIVISOR)) {
}

void CostWrapper::set_durations_matrix(const Matrix<Cost>* matrix) {
  cost_matrix_size = matrix->size();
  cost_data = (*matrix)[0];
}

} // namespace vroom
