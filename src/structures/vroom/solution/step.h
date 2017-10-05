#ifndef STEP_H
#define STEP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../location.h"

enum class TYPE { START, JOB, END };

struct step {
  TYPE type;
  location_t location;
  ID_t job;

  step(TYPE type, location_t location);

  step(TYPE type, location_t location, ID_t job);
};

#endif
