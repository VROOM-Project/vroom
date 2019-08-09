/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/raw_route.h"

namespace vroom {

RawRoute::RawRoute(const Input& input, Index i)
  : vehicle_rank(i),
    has_start(input.vehicles[i].has_start()),
    has_end(input.vehicles[i].has_end()) {
}

void RawRoute::set_route(const std::vector<Index>& r) {
  route = r;
}

bool RawRoute::empty() const {
  return route.empty();
}

std::size_t RawRoute::size() const {
  return route.size();
}

void RawRoute::add(const Input&, const Index job_rank, const Index rank) {
  route.insert(route.begin() + rank, job_rank);
}

void RawRoute::remove(const Input&, const Index rank, const unsigned count) {
  route.erase(route.begin() + rank, route.begin() + rank + count);
}

} // namespace vroom
