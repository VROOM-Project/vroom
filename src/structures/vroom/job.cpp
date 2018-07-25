/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/job.h"

job_t::job_t(ID_t id,
             const location_t& location,
             duration_t service,
             const amount_t& amount,
             const std::unordered_set<skill_t>& skills,
             const std::vector<time_window_t>& tws)
  : id(id),
    location(location),
    service(service),
    amount(amount),
    skills(skills),
    tws(tws) {
}

index_t job_t::index() const {
  return location.index();
}
