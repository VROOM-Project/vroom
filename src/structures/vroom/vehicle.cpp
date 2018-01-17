/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "vehicle.h"

vehicle_t::vehicle_t(ID_t id,
                     const boost::optional<location_t>& start,
                     const boost::optional<location_t>& end)
  : vehicle_t(id, start, end, boost::none) {
}

vehicle_t::vehicle_t(ID_t id,
                     const boost::optional<location_t>& start,
                     const boost::optional<location_t>& end,
                     const boost::optional<amount_t>& capacity)
  : id(id), start(start), end(end), capacity(capacity) {
  if ((start == boost::none) and (end == boost::none)) {
    throw custom_exception("No start or end specified for vehicle " +
                           std::to_string(id) + '.');
  }
}

bool vehicle_t::has_start() const {
  return start != boost::none;
}

bool vehicle_t::has_end() const {
  return end != boost::none;
}

bool vehicle_t::has_capacity() const {
  return capacity != boost::none;
}
