#ifndef RAW_ROUTE_H
#define RAW_ROUTE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"

namespace vroom {

class RawRoute {
public:
  Index vehicle_rank;
  bool has_start;
  bool has_end;

  std::vector<Index> route;

  RawRoute(const Input& input, Index i);

  void set_route(const std::vector<Index>& r);

  bool empty() const;

  std::size_t size() const;

  bool is_valid_addition_for_tw(const Input&, const Index, const Index) const {
    return true;
  };

  void add(const Input&, const Index job_rank, const Index rank);

  bool is_valid_removal(const Input&, const Index, const unsigned) const {
    return true;
  };

  void remove(const Input&, const Index rank, const unsigned count);
};

} // namespace vroom

#endif
