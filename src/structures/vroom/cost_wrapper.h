#ifndef COST_WRAPPER_H
#define COST_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/generic/matrix.h"
#include "structures/typedefs.h"

namespace vroom {

struct CostWrapper {
  const static uint32_t DIVISOR = 100;

  const uint32_t discrete_factor;
  std::size_t cost_matrix_size;
  const Cost* cost_data;

  CostWrapper(double speed_factor);

  void set_durations_matrix(const Matrix<Cost>* matrix);

  Cost cost(Index i, Index j) const {
    Cost c = cost_data[i * cost_matrix_size + j];
    return (c * discrete_factor) / DIVISOR;
  }
};

} // namespace vroom

#endif
