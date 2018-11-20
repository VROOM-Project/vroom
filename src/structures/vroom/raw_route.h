#ifndef RAW_ROUTE_H
#define RAW_ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"

class raw_route {
public:
  std::vector<index_t> route;

  raw_route() : route(){};

  raw_route(const std::vector<index_t>& r) : route(r){};

  raw_route(const input&, index_t) : route(){};

  bool empty() const {
    return route.empty();
  }

  std::size_t size() const {
    return route.size();
  }

  bool is_valid_addition_for_tw(const input&,
                                const index_t,
                                const index_t) const {
    return true;
  };

  void add(const input&, const index_t job_rank, const index_t rank) {
    route.insert(route.begin() + rank, job_rank);
  };

  bool is_valid_removal(const input&, const index_t, const unsigned) const {
    return true;
  };

  void remove(const input&, const index_t rank, const unsigned count) {
    route.erase(route.begin() + rank, route.begin() + rank + count);
  };
};

#endif
