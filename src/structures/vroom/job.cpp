/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>

#include "structures/vroom/job.h"
#include "utils/exception.h"

namespace vroom {

Job::Job(Id id,
         const Location& location,
         Duration service,
         const Amount& amount,
         const Skills& skills,
         const std::vector<TimeWindow>& tws)
  : id(id),
    location(location),
    service(service),
    amount(amount),
    skills(skills),
    tws(tws),
    tw_length(
      std::accumulate(std::next(tws.begin()),
                      tws.end(),
                      tws[0].length,
                      [](auto sum, auto tw) { return sum + tw.length; })) {
  if (tws.size() == 0) {
    throw Exception(ERROR::INPUT,
                    "Empty time-windows for job " + std::to_string(id) + ".");
  }

  if (tws.size() > 1) {
    for (std::size_t i = 0; i < tws.size() - 1; ++i) {
      if (tws[i + 1].start <= tws[i].end) {
        throw Exception(ERROR::INPUT,
                        "Unsorted or overlapping time-windows for job " +
                          std::to_string(id) + ".");
      }
    }
  }
}

Index Job::index() const {
  return location.index();
}

bool Job::is_valid_start(Duration time) const {
  bool valid = false;

  for (const auto& tw : tws) {
    if (tw.contains(time)) {
      valid = true;
      break;
    }
  }

  return valid;
}

} // namespace vroom
