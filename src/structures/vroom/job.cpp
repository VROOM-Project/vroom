/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "job.h"

job_t::job_t(ID_t id,
             const location_t& location,
             const amount_t& amount,
             const std::unordered_set<skill_t>& skills,
             duration_t service)
  : id(id),
    location(location),
    amount(amount),
    skills(skills),
    service(service) {
}

index_t job_t::index() const {
  return location.index();
}
