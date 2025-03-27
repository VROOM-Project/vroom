#ifndef COMPUTING_TIMES_H
#define COMPUTING_TIMES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom {

struct ComputingTimes {
  // Computing times in milliseconds.
  UserDuration loading{0};
  UserDuration solving{0};
  UserDuration routing{0};

  ComputingTimes();
};

} // namespace vroom

#endif
