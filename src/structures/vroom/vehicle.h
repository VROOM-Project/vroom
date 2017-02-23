#ifndef VEHICLE_H
#define VEHICLE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../typedefs.h"
#include "./location.h"

struct vehicle {
  const index_t id;
  boost::optional<location_t> start;
  boost::optional<location_t> end;
  boost::optional<index_t> start_id;
  boost::optional<index_t> end_id;

  vehicle(index_t id,
          boost::optional<location_t> start,
          boost::optional<location_t> end):
    id(id),
    start(start),
    end(end),
    start_id(boost::none),
    end_id(boost::none){}
  
  vehicle(index_t id,
          boost::optional<index_t> start_id,
          boost::optional<index_t> end_id):
    id(id),
    start(boost::none),
    end(boost::none),
    start_id(start_id),
    end_id(end_id){}

  vehicle(index_t id,
          boost::optional<location_t> start,
          boost::optional<location_t> end,
          boost::optional<index_t> start_id,
          boost::optional<index_t> end_id):
    id(id),
    start(start),
    end(end),
    start_id(start_id),
    end_id(end_id){}

  bool has_start() const{
    return start != boost::none || start_id;
  }

  bool has_end() const{
    return end != boost::none || end_id;
  }
};

#endif
