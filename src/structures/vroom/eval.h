#ifndef EVAL_H
#define EVAL_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom {

struct Eval {
  Cost cost;
  Duration duration;

  Eval() : cost(0), duration(0){};

  Eval(Cost cost, Duration duration) : cost(cost), duration(duration){};

  Eval& operator+=(const Eval& rhs) {
    cost += rhs.cost;
    duration += rhs.duration;

    return *this;
  }

  friend Eval operator+(Eval lhs, const Eval& rhs) {
    lhs += rhs;
    return lhs;
  }
};

} // namespace vroom

#endif
