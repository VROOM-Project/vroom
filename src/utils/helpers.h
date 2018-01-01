#ifndef HELPERS_H
#define HELPERS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../structures/typedefs.h"
#include "./exceptions.h"

inline cost_t add_without_overflow(cost_t a, cost_t b) {
  if (a > std::numeric_limits<cost_t>::max() - b) {
    throw custom_exception(
      "Too high cost values, stopping to avoid overflowing.");
  }
  return a + b;
}

#endif
