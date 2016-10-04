#ifndef VEHICLE_H
#define VEHICLE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <boost/optional.hpp>
#include "../typedefs.h"
#include "./location.h"

struct vehicle{
  const index_t id;
  boost::optional<location> start;
  boost::optional<location> end;

  vehicle(index_t id,
          boost::optional<location> start,
          boost::optional<location> end):
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
