#ifndef VEHICLE_H
#define VEHICLE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../typedefs.h"
#include "./location.h"

struct vehicle{
  const index_t id;
  boost::optional<location_t> start;
  boost::optional<location_t> end;

  vehicle(index_t id,
          boost::optional<location_t> start,
          boost::optional<location_t> end):
    id(id),
    start(start),
    end(end){}

  bool has_start() const{
    return start != boost::none;
  }

  bool has_end() const{
    return end != boost::none;
  }
};

#endif
