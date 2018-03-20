#ifndef VEHICLE_H
#define VEHICLE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_set>

#include "../../utils/exceptions.h"
#include "../typedefs.h"
#include "./amount.h"
#include "./location.h"

struct vehicle_t {
  const ID_t id;
  boost::optional<location_t> start;
  boost::optional<location_t> end;
  boost::optional<amount_t> capacity;
  std::unordered_set<skill_t> skills;

  vehicle_t(ID_t id,
            const boost::optional<location_t>& start,
            const boost::optional<location_t>& end);

  vehicle_t(ID_t id,
            const boost::optional<location_t>& start,
            const boost::optional<location_t>& end,
            const boost::optional<amount_t>& capacity,
            const std::unordered_set<skill_t>& skills);

  bool has_start() const;

  bool has_end() const;

  bool has_capacity() const;
};

#endif
