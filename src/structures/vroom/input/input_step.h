#ifndef INPUT_STEP_H
#define INPUT_STEP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom {

struct InputStep {
  const Id id;
  const JOB_TYPE type;
};

} // namespace vroom

#endif
