#ifndef EVAL_H
#define EVAL_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <tuple>

#include "structures/typedefs.h"

namespace vroom {

struct Eval {
  Cost cost;
  Duration duration;
  Distance distance;

  constexpr Eval() : cost(0), duration(0), distance(0){};

  constexpr Eval(Cost cost, Duration duration = 0, Distance distance = 0)
    : cost(cost), duration(duration), distance(distance){};

  Eval& operator+=(const Eval& rhs) {
    cost += rhs.cost;
    duration += rhs.duration;
    distance += rhs.distance;

    return *this;
  }

  Eval& operator-=(const Eval& rhs) {
    cost -= rhs.cost;
    duration -= rhs.duration;
    distance -= rhs.distance;

    return *this;
  }

  Eval operator-() const {
    return {-cost, -duration, -distance};
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
    return std::tie(lhs.cost, lhs.duration, lhs.distance) <
           std::tie(rhs.cost, rhs.duration, rhs.distance);
  }

  friend bool operator<=(const Eval& lhs, const Eval& rhs) {
    return lhs.cost <= rhs.cost;
  }

  friend bool operator==(const Eval& lhs, const Eval& rhs) {
    return lhs.cost == rhs.cost && lhs.duration == rhs.duration &&
           lhs.distance == rhs.distance;
  }

  friend bool operator!=(const Eval& lhs, const Eval& rhs) {
    return lhs.cost != rhs.cost || lhs.duration != rhs.duration ||
           lhs.distance != rhs.distance;
  }
};

constexpr Eval NO_EVAL = {std::numeric_limits<Cost>::max(), 0, 0};
constexpr Eval NO_GAIN = {std::numeric_limits<Cost>::min(), 0, 0};

} // namespace vroom

#endif
