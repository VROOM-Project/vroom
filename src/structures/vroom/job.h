#ifndef JOB_H
#define JOB_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_set>

#include "../typedefs.h"
#include "./amount.h"
#include "./location.h"

struct job_t {
  const ID_t id;
  location_t location;
  amount_t amount;
  std::unordered_set<skill_t> skills;

  job_t(ID_t id,
        location_t location,
        const amount_t& amount,
        const std::unordered_set<skill_t>& skills);

  index_t index() const;
};

#endif
