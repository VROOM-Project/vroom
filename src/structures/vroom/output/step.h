#ifndef STEP_H
#define STEP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./location.h"

enum class TYPE {START, JOB, END};

struct step{
  const TYPE type;
  const location location;
  const boost::optional<index_t> job;

  step(TYPE type,
       location location):
    type(type),
    location(location){}

  step(TYPE type,
       location location,
       index_t job):
    step(type, location),
    job(job){}
};

#endif
