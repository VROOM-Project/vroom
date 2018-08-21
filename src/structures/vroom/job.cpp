/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/job.h"
#include "utils/exceptions.h"

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
  if (tws.size() > 1) {
    for (std::size_t i = 0; i < tws.size() - 1; ++i) {
      if (tws[i + 1].start <= tws[i].end) {
        throw custom_exception("Unsorted or overlapping time-windows for job " +
                               std::to_string(id) + ".");
      }
    }
  }
}

index_t job_t::index() const {
  return location.index();
}
