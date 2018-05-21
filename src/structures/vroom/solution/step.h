#ifndef STEP_H
#define STEP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../job.h"
#include "../location.h"

struct step {
  const TYPE type;
  const location_t location;
  const ID_t job;
  const duration_t service;
  const amount_t amount;

  duration_t arrival;
  duration_t duration;
  distance_t distance;

  step(TYPE type, location_t location);

  step(const job_t& job);
};

#endif
