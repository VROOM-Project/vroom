/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/cost_wrapper.h"
#include "utils/exception.h"
#include <cmath>

namespace vroom {

CostWrapper::CostWrapper(double speed_factor, Cost per_hour)
  : discrete_duration_factor(std::round(1 / speed_factor * DURATION_FACTOR)),
    discrete_cost_factor(
      std::round(1 / speed_factor * DURATION_FACTOR * per_hour)),
    _speed_factor(speed_factor),
    _per_hour(per_hour),
    _cost_based_on_duration(true) {
  if (speed_factor <= 0 || speed_factor > MAX_SPEED_FACTOR) {
    throw InputException("Invalid speed factor: " +
                         std::to_string(speed_factor));
  }
}

void CostWrapper::set_durations_matrix(const Matrix<UserDuration>* matrix) {
  duration_matrix_size = matrix->size();
  duration_data = (*matrix)[0];
}

void CostWrapper::set_costs_matrix(const Matrix<UserCost>* matrix,
                                   bool reset_cost_factor) {
  cost_matrix_size = matrix->size();
  cost_data = (*matrix)[0];

  if (reset_cost_factor) {
    discrete_cost_factor = DURATION_FACTOR * COST_FACTOR;
    _cost_based_on_duration = false;
  }
}

UserCost CostWrapper::user_cost_from_user_duration(UserDuration d) const {
  assert(_cost_based_on_duration);
  return (d * _per_hour) / COST_FACTOR;
}

} // namespace vroom
