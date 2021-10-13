/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/cost_wrapper.h"
#include "utils/exception.h"
#include <cmath>

namespace vroom {

CostWrapper::CostWrapper(double speed_factor)
  : discrete_duration_factor(std::round(1 / speed_factor * DIVISOR)),
    discrete_cost_factor(std::round(1 / speed_factor * DIVISOR)) {
  if (speed_factor <= 0 || speed_factor > MAX_SPEED_FACTOR) {
    throw Exception(ERROR::INPUT,
                    "Invalid speed factor: " + std::to_string(speed_factor));
  }
}

void CostWrapper::set_durations_matrix(const Matrix<Duration>* matrix) {
  duration_matrix_size = matrix->size();
  duration_data = (*matrix)[0];
}

void CostWrapper::set_costs_factor(double cost_factor) {
  discrete_cost_factor = std::round(cost_factor * DIVISOR);
}

void CostWrapper::set_costs_matrix(const Matrix<Cost>* matrix) {
  cost_matrix_size = matrix->size();
  cost_data = (*matrix)[0];
}

} // namespace vroom
