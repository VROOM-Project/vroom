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
  const STEP_TYPE type;
  const JOB_TYPE job_type;
  // Stores rank of current step (in input.jobs vector for a
  // job/pickup/delivery and in vehicle.breaks for a break).
  Index rank;

  // Used for start and end.
  InputStep(STEP_TYPE type);

  // Used for breaks.
  InputStep(STEP_TYPE type, Id id);

  // Used for single jobs, pickups and deliveries.
  InputStep(JOB_TYPE job_type, Id id);
};

} // namespace vroom

#endif
