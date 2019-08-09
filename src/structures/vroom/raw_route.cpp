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
    has_end(input.vehicles[i].has_end()),
    capacity(input.vehicles[i].capacity) {
}

void RawRoute::set_route(const Input& input, const std::vector<Index>& r) {
  route = r;
  update_amounts(input);
}

bool RawRoute::empty() const {
  return route.empty();
}

std::size_t RawRoute::size() const {
  return route.size();
}

void RawRoute::update_amounts(const Input& input) {
  auto step_size = route.empty() ? 0 : route.size() + 2;
  fwd_pickups.resize(route.size());
  bwd_deliveries.resize(route.size());
  current_loads.resize(step_size);
  fwd_peaks.resize(step_size);
  bwd_peaks.resize(step_size);

  if (route.empty()) {
    // So that check in is_valid_addition_for_capacity is consistent
    // with empty routes.
    std::fill(fwd_peaks.begin(), fwd_peaks.end(), Amount(input.amount_size()));
    std::fill(bwd_peaks.begin(), bwd_peaks.end(), Amount(input.amount_size()));
    return;
  }

  Amount current_pickups(input.amount_size());

  for (std::size_t i = 0; i < route.size(); ++i) {
    current_pickups += input.jobs[route[i]].pickup;
    fwd_pickups[i] = current_pickups;
  }

  Amount current_deliveries(input.amount_size());

  current_loads.back() = fwd_pickups.back();

  for (std::size_t i = 0; i < route.size(); ++i) {
    auto bwd_i = route.size() - i - 1;

    bwd_deliveries[bwd_i] = current_deliveries;
    current_loads[bwd_i + 1] = fwd_pickups[bwd_i] + current_deliveries;
    current_deliveries += input.jobs[route[bwd_i]].delivery;
  }
  current_loads[0] = current_deliveries;

  auto peak = current_loads[0];
  fwd_peaks[0] = peak;
  for (std::size_t s = 1; s < fwd_peaks.size(); ++s) {
    // Handle max component-wise.
    for (std::size_t r = 0; r < input.amount_size(); ++r) {
      peak[r] = std::max(peak[r], current_loads[s][r]);
    }
    fwd_peaks[s] = peak;
  }

  peak = current_loads.back();
  bwd_peaks.back() = peak;
  for (std::size_t s = 1; s < bwd_peaks.size(); ++s) {
    auto bwd_s = bwd_peaks.size() - s - 1;
    // Handle max component-wise.
    for (std::size_t r = 0; r < input.amount_size(); ++r) {
      peak[r] = std::max(peak[r], current_loads[bwd_s][r]);
    }
    bwd_peaks[bwd_s] = peak;
  }
}

bool RawRoute::is_valid_addition_for_capacity(const Input&,
                                              const Amount& pickup,
                                              const Amount& delivery,
                                              const Index rank) const {
  assert(rank <= route.size());

  return (fwd_peaks[rank] + delivery <= capacity) and
         (bwd_peaks[rank] + pickup <= capacity);
}

Amount RawRoute::get_load(Index s) const {
  return current_loads[s];
}

void RawRoute::add(const Input& input, const Index job_rank, const Index rank) {
  route.insert(route.begin() + rank, job_rank);
  update_amounts(input);
}

void RawRoute::remove(const Input& input,
                      const Index rank,
                      const unsigned count) {
  route.erase(route.begin() + rank, route.begin() + rank + count);
  update_amounts(input);
}

} // namespace vroom
