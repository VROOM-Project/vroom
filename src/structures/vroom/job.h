#ifndef JOB_H
#define JOB_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/location.h"
#include "structures/vroom/time_window.h"

struct job_t {
  const ID_t id;
  location_t location;
  const duration_t service;
  const amount_t amount;
  const skills_t skills;
  const std::vector<time_window_t> tws;

  job_t(ID_t id,
        const location_t& location,
        duration_t service = 0,
        const amount_t& amount = amount_t(0),
        const skills_t& skills = skills_t(),
        // TODO handle default.
        const std::vector<time_window_t>& tws = std::vector<time_window_t>());

  index_t index() const;
};

#endif
