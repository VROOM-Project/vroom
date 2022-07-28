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
  SignedCost cost;
  SignedDuration duration;

  constexpr Eval() : cost(0), duration(0){};

  constexpr Eval(Cost cost, Duration duration)
    : cost(cost), duration(duration){};

  constexpr Eval(SignedCost cost, SignedDuration duration)
    : cost(cost), duration(duration){};

  Eval& operator+=(const Eval& rhs) {
    cost += rhs.cost;
    duration += rhs.duration;

    return *this;
  }

  Eval& operator-=(const Eval& rhs) {
    cost -= rhs.cost;
    duration -= rhs.duration;

    return *this;
  }

  friend Eval operator+(Eval lhs, const Eval& rhs) {
    lhs += rhs;
    return lhs;
  }

  friend Eval operator-(Eval lhs, const Eval& rhs) {
    lhs -= rhs;
    return lhs;
  }

  friend bool operator<(const Eval& lhs, const Eval& rhs) {
    return lhs.cost < rhs.cost;
  }

  friend bool operator<=(const Eval& lhs, const Eval& rhs) {
    return lhs.cost <= rhs.cost;
  }

  friend bool operator==(const Eval& lhs, const Eval& rhs) {
    return lhs.cost == rhs.cost and lhs.duration == rhs.duration;
  }

  friend bool operator!=(const Eval& lhs, const Eval& rhs) {
    return lhs.cost != rhs.cost or lhs.duration != rhs.duration;
  }
};

constexpr Eval NO_EVAL = {std::numeric_limits<SignedCost>::max(), 0};

} // namespace vroom

#endif
