#ifndef COST_WRAPPER_H
#define COST_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/generic/matrix.h"
#include "structures/typedefs.h"

namespace vroom {

class CostWrapper {
private:
  const Cost _per_hour;
  const Cost _per_km;

  // Used to scale durations internally in order to account for
  // vehicle speed_factor.
  const Duration discrete_duration_factor;

  // Used to consistently ponder durations and distances as cost
  // values.
  Cost discrete_duration_cost_factor;
  Cost discrete_distance_cost_factor;

  std::size_t duration_matrix_size;
  const UserDuration* duration_data;

  std::size_t distance_matrix_size;
  const UserDistance* distance_data;

  std::size_t cost_matrix_size;
  const UserCost* cost_data;

  bool _cost_based_on_metrics{true};

public:
  CostWrapper(double speed_factor, Cost per_hour, Cost per_km);

  void set_durations_matrix(const Matrix<UserDuration>* matrix);

  void set_distances_matrix(const Matrix<UserDistance>* matrix);

  void set_costs_matrix(const Matrix<UserCost>* matrix,
                        bool reset_cost_factor = false);

  bool cost_based_on_metrics() const {
    return _cost_based_on_metrics;
  }

  bool has_same_variable_costs(const CostWrapper& other) const {
    return (this->discrete_duration_cost_factor ==
            other.discrete_duration_cost_factor) &&
           (this->discrete_distance_cost_factor ==
            other.discrete_distance_cost_factor);
  }

  Duration duration(Index i, Index j) const {
    return discrete_duration_factor *
           static_cast<Duration>(duration_data[i * duration_matrix_size + j]);
  }

  Distance distance(Index i, Index j) const {
    return static_cast<Distance>(distance_data[i * distance_matrix_size + j]);
  }

  Cost cost(Index i, Index j) const {
    // If custom costs are provided, this boils down to scaling the
    // actual costs. If costs are computed from travel times and
    // distances, then cost_data holds the travel times so we ponder
    // costs based on per_hour and per_km.
    return discrete_duration_cost_factor *
             static_cast<Cost>(cost_data[i * cost_matrix_size + j]) +
           discrete_distance_cost_factor *
             static_cast<Cost>(distance_data[i * distance_matrix_size + j]);
  }

  UserCost user_cost_from_user_metrics(UserDuration d, UserDistance m) const;
};

} // namespace vroom

#endif
