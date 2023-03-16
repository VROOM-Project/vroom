#ifndef COST_WRAPPER_H
#define COST_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/generic/matrix.h"
#include "structures/typedefs.h"

namespace vroom {

class CostWrapper {
private:
  const Duration discrete_duration_factor;
  std::size_t duration_matrix_size;
  const UserDuration* duration_data;

  Cost discrete_cost_factor;
  std::size_t cost_matrix_size;
  const UserCost* cost_data;

  const double _speed_factor;
  Cost _per_hour;
  bool _cost_based_on_duration;

public:
  CostWrapper(double speed_factor, Cost per_hour);

  void set_durations_matrix(const Matrix<UserDuration>* matrix);

  void set_costs_matrix(const Matrix<UserCost>* matrix,
                        bool reset_cost_factor = false);

  Duration get_discrete_duration_factor() const {
    return discrete_duration_factor;
  }

  bool cost_based_on_duration() const {
    return _cost_based_on_duration;
  }

  Duration duration(Index i, Index j) const {
    return discrete_duration_factor *
           static_cast<Duration>(duration_data[i * duration_matrix_size + j]);
  }

  Cost cost(Index i, Index j) const {
    return discrete_cost_factor *
           static_cast<Cost>(cost_data[i * cost_matrix_size + j]);
  }

  double get_speed_factor() const {
    return _speed_factor;
  }

  Cost get_per_hour() const {
    return _per_hour;
  }

  UserCost user_cost_from_user_duration(UserDuration d) const;
};

} // namespace vroom

#endif
