/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/break.h"
#include "utils/helpers.h"

namespace vroom {

Break::Break(Id id,
             const std::vector<TimeWindow>& tws,
             Duration service,
             const std::string& description,
             const std::optional<Amount>& max_load)
  : id(id),
    tws(tws),
    service(service),
    description(description),
    max_load(max_load) {
  utils::check_tws(tws);
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
  return !max_load.has_value() or load <= max_load.value();
}

} // namespace vroom
