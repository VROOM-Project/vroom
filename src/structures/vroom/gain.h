#ifndef GAIN_H
#define GAIN_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/eval.h"

namespace vroom {

struct Gain {
  SignedCost cost;
  SignedDuration duration;

  Gain() : cost(0), duration(0){};

  Gain(SignedCost cost, SignedDuration duration)
    : cost(cost), duration(duration){};

  Gain(Eval eval) : cost(eval.cost), duration(eval.duration){};

  Gain& operator+=(const Gain& rhs) {
    cost += rhs.cost;
    duration += rhs.duration;

    return *this;
  }

  Gain& operator-=(const Gain& rhs) {
    cost -= rhs.cost;
    duration -= rhs.duration;

    return *this;
  }

  friend Gain operator+(Gain lhs, const Gain& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Gain operator-(Gain lhs, const Gain& rhs) {
    lhs -= rhs;
    return lhs;
  }
};

} // namespace vroom

#endif
