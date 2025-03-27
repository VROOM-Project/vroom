/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/break.h"
#include "utils/helpers.h"

namespace vroom {

Break::Break(Id id,
             const std::vector<TimeWindow>& tws,
             UserDuration service,
             std::string description,
             std::optional<Amount> max_load)
  : id(id),
    tws(tws),
    service(utils::scale_from_user_duration(service)),
    description(std::move(description)),
    max_load(std::move(max_load)) {
  utils::check_tws(tws, id, "break");
}

bool Break::is_valid_start(Duration time) const {
  bool valid = false;

  for (const auto& tw : tws) {
    if (tw.contains(time)) {
      valid = true;
      break;
    }
  }

  return valid;
}

bool Break::is_valid_for_load(const Amount& load) const {
  return !max_load.has_value() || load <= max_load.value();
}

} // namespace vroom
