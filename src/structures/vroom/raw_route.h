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

namespace vroom {

class RawRoute {
public:
  std::vector<Index> route;

  RawRoute() : route(){};

  RawRoute(const std::vector<Index>& r) : route(r){};

  RawRoute(const Input&, Index) : route(){};

  bool empty() const {
    return route.empty();
  }

  std::size_t size() const {
    return route.size();
  }

  bool is_valid_addition_for_tw(const Input&, const Index, const Index) const {
    return true;
  };

  void add(const Input&, const Index job_rank, const Index rank) {
    route.insert(route.begin() + rank, job_rank);
  };

  bool is_valid_removal(const Input&, const Index, const unsigned) const {
    return true;
  };

  void remove(const Input&, const Index rank, const unsigned count) {
    route.erase(route.begin() + rank, route.begin() + rank + count);
  };
};

} // namespace vroom

#endif
