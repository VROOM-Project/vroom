/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/vehicle.h"
#include "utils/exceptions.h"

vehicle_t::vehicle_t(ID_t id,
                     const boost::optional<location_t>& start,
                     const boost::optional<location_t>& end,
                     const amount_t& capacity,
                     const std::unordered_set<skill_t>& skills,
                     const time_window_t& tw)
  : id(id), start(start), end(end), capacity(capacity), skills(skills), tw(tw) {
  if (!static_cast<bool>(start) and !static_cast<bool>(end)) {
    throw custom_exception("No start or end specified for vehicle " +
                           std::to_string(id) + '.');
  }
}

bool vehicle_t::has_start() const {
  return static_cast<bool>(start);
}

bool vehicle_t::has_end() const {
  return static_cast<bool>(end);
}
