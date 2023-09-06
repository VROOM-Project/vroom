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

  constexpr Eval() : cost(0), duration(0){};

  constexpr Eval(Cost cost, Duration duration)
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

  Eval operator-() const {
    return {-cost, -duration};
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
    return lhs.cost < rhs.cost or
           (lhs.cost == rhs.cost and lhs.duration < rhs.duration);
  }

  friend bool operator>(const Eval& lhs, const Eval& rhs) {
    return lhs.cost > rhs.cost;
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

constexpr Eval MAX_EVAL = {std::numeric_limits<Cost>::max(),
                           std::numeric_limits<Duration>::max()};
constexpr Eval NO_EVAL = {std::numeric_limits<Cost>::max(), 0};
constexpr Eval NO_GAIN = {std::numeric_limits<Cost>::min(), 0};

} // namespace vroom

#endif
