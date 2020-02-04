#ifndef VEHICLE_H
#define VEHICLE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/location.h"
#include "structures/vroom/time_window.h"

namespace vroom {

struct Vehicle {
  const Id id;
  boost::optional<Location> start;
  boost::optional<Location> end;
  const Amount capacity;
  const Skills skills;
  const TimeWindow tw;

  Vehicle(Id id,
          const boost::optional<Location>& start,
          const boost::optional<Location>& end,
          const Amount& capacity = Amount(0),
          const Skills& skills = Skills(),
          const TimeWindow& tw = TimeWindow());

  bool has_start() const;

  bool has_end() const;

  bool has_same_locations(const Vehicle& other) const;
};

} // namespace vroom

#endif
