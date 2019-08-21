/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/raw_route.h"

namespace vroom {

RawRoute::RawRoute(const Input& input, Index i)
  : fwd_peaks(2, Amount(input.amount_size())),
    bwd_peaks(2, Amount(input.amount_size())),
    vehicle_rank(i),
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
  auto step_size = route.size() + 2;
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

  for (const auto& load : current_loads) {
    // TODO remove
    assert(load <= capacity);
  }
}

bool RawRoute::is_valid_addition_for_capacity(const Input&,
                                              const Amount& pickup,
                                              const Amount& delivery,
                                              const Index rank) const {
  assert(rank <= route.size());

  assert(fwd_peaks.size() == route.size() + 2);

  return (fwd_peaks[rank] + delivery <= capacity) and
         (bwd_peaks[rank] + pickup <= capacity);
}

template <class InputIterator>
bool RawRoute::is_valid_addition_for_capacity(const Input& input,
                                              const Amount& pickup,
                                              const Amount& delivery,
                                              InputIterator first_job,
                                              InputIterator last_job,
                                              const Index first_rank,
                                              const Index last_rank) const {
  assert(first_rank < last_rank);
  assert(last_rank <= route.size() + 1);

  auto first_deliveries =
    (first_rank == 0) ? current_loads[0] : bwd_deliveries[first_rank - 1];

  auto first_pickups = (first_rank == 0) ? Amount(input.amount_size())
                                         : fwd_pickups[first_rank - 1];

  auto replaced_deliveries = first_deliveries - bwd_deliveries[last_rank - 1];

  bool valid =
    (fwd_peaks[first_rank] + delivery <= capacity + replaced_deliveries) and
    (bwd_peaks[last_rank] + pickup <=
     capacity + fwd_pickups[last_rank - 1] - first_pickups);

  if (valid) {
    Amount current_load =
      current_loads[first_rank] - replaced_deliveries + delivery;

    for (auto job_iter = first_job; valid and job_iter != last_job;
         ++job_iter) {
      auto& job = input.jobs[*job_iter];
      current_load += job.pickup;
      current_load -= job.delivery;

      valid &= (current_load <= capacity);
    }
  }

  return valid;
}

Amount RawRoute::get_load(Index s) const {
  return current_loads[s];
}

Amount RawRoute::pickup_in_range(Index i, Index j) const {
  assert(i < j);
  if (i == 0) {
    return fwd_pickups[j - 1];
  } else {
    return fwd_pickups[j - 1] - fwd_pickups[i - 1];
  }
}

Amount RawRoute::delivery_in_range(Index i, Index j) const {
  assert(i < j);
  auto before_deliveries = (i == 0) ? current_loads[0] : bwd_deliveries[i - 1];
  return before_deliveries - bwd_deliveries[j - 1];
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

template bool
RawRoute::is_valid_addition_for_capacity(const Input& input,
                                         const Amount& pickup,
                                         const Amount& delivery,
                                         std::vector<Index>::iterator first_job,
                                         std::vector<Index>::iterator last_job,
                                         const Index first_rank,
                                         const Index last_rank) const;
template bool RawRoute::is_valid_addition_for_capacity(
  const Input& input,
  const Amount& pickup,
  const Amount& delivery,
  std::vector<Index>::reverse_iterator first_job,
  std::vector<Index>::reverse_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;

} // namespace vroom
