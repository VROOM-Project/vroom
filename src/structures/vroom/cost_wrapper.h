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
  const double durations_factor;
  const Matrix<Cost>* durations_matrix;

  CostWrapper(double speed_factor);

  void set_durations_matrix(const Matrix<Cost>* matrix);
};

} // namespace vroom

#endif
