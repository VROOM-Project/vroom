#ifndef JOB_H
#define JOB_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../typedefs.h"
#include "./amount.h"
#include "./location.h"

struct job_t {
  const ID_t id;
  location_t location;
  const amount_t amount;
  const skills_t skills;
  const duration_t service;

  job_t(ID_t id,
        const location_t& location,
        const amount_t& amount = amount_t(0),
        const skills_t& skills = skills_t(),
        duration_t service = 0);

  index_t index() const;
};

#endif
