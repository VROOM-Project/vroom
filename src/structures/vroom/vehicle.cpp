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

bool vehicle_t::has_same_locations(const vehicle_t& other) const {
  bool same = (this->has_start() == other.has_start()) and
              (this->has_end() == other.has_end());

  if (same and this->has_start()) {
    same &= this->start.get() == other.start.get();
  }

  if (same and this->has_end()) {
    same &= this->end.get() == other.end.get();
  }

  return same;
}
