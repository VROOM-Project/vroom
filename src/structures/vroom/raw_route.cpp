/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/raw_route.h"
#include "utils/helpers.h"

namespace vroom {

RawRoute::RawRoute(const Input& input, Index v)
  : _zero(input.get_amount_size()),
    _fwd_peaks(2, _zero),
    _bwd_peaks(2, _zero),
    _delivery_margin(input.vehicles[v].capacity),
    _pickup_margin(input.vehicles[v].capacity),
    v_rank(v),
    v_type(input.vehicles[v].type),
    has_start(input.vehicles[v].has_start()),
    has_end(input.vehicles[v].has_end()),
    capacity(input.vehicles[v].capacity) {
}

void RawRoute::populate_from_steps(const Input& input) {
  const auto& vehicle = input.vehicles[v_rank];
  if (!vehicle.steps.empty()) {
    const auto route_data = check_route_steps(input);

    if (!route_data.job_ranks.empty()) {
      // Proceed with updating current route and all amounts.
      set_route(input, route_data.job_ranks);
    }
  }
}

InitRouteData RawRoute::check_route_steps(const Input& input) const {
  // Check that provided route is OK with regard to capacity,
  // max_travel_time, max_tasks, precedence and skills constraints.
  const auto& vehicle = input.vehicles[v_rank];
  assert(!vehicle.steps.empty());

  // Startup load is the sum of deliveries for (single) jobs.
  InitRouteData route_data(vehicle.steps.size());
  route_data.single_jobs_deliveries =
    utils::get_single_jobs_deliveries(input, vehicle.steps);

  if (!(route_data.single_jobs_deliveries <= vehicle.capacity)) {
    throw InputException(
      std::format("Route over capacity for vehicle {}.", vehicle.id));
  }

  // Track load and travel time during the route for validity.
  Amount current_load = route_data.single_jobs_deliveries;
  Eval eval_sum;
  std::optional<Index> previous_index;
  if (vehicle.has_start()) {
    previous_index = vehicle.start.value().index();
  }

  // Initialize first break count slots.
  route_data.breaks_at_rank.push_back(0);
  route_data.breaks_counts.push_back(0);

  std::unordered_set<Index> expected_delivery_ranks;
  for (const auto& step : vehicle.steps) {
    // Filter for real jobs while populating break data.
    if (step.type != STEP_TYPE::JOB) {
      if (step.type == STEP_TYPE::BREAK) {
        route_data.breaks_at_rank.back() += 1;
        route_data.breaks_counts.back() += 1;
      }
      continue;
    }

    const auto job_rank = step.rank;
    const auto& job = input.jobs[job_rank];
    route_data.job_ranks.push_back(job_rank);
    route_data.breaks_at_rank.push_back(0);
    route_data.breaks_counts.push_back(route_data.breaks_counts.back());

    if (!input.vehicle_ok_with_job(v_rank, job_rank)) {
      throw InputException(
        std::format("Missing skill or step out of reach for vehicle {} and "
                    "job {}.",
                    vehicle.id,
                    job.id));
    }

    // Update current travel time.
    if (previous_index.has_value()) {
      eval_sum += vehicle.eval(previous_index.value(), job.index());
    }
    previous_index = job.index();

    // Handle load.
    assert(step.job_type.has_value());
    switch (step.job_type.value()) {
    case JOB_TYPE::SINGLE: {
      current_load += job.pickup;
      current_load -= job.delivery;
      break;
    }
    case JOB_TYPE::PICKUP: {
      expected_delivery_ranks.insert(job_rank + 1);

      current_load += job.pickup;
      break;
    }
    case JOB_TYPE::DELIVERY: {
      auto search = expected_delivery_ranks.find(job_rank);
      if (search == expected_delivery_ranks.end()) {
        throw InputException(
          std::format("Invalid shipment in route for vehicle {}.", vehicle.id));
      }
      expected_delivery_ranks.erase(search);

      current_load -= job.delivery;
      break;
    }
    default:
      assert(false);
    }

    // Check validity after this step wrt capacity.
    if (!(current_load <= vehicle.capacity)) {
      throw InputException(
        std::format("Route over capacity for vehicle {}.", vehicle.id));
    }
  }
  assert(route_data.breaks_at_rank.size() == route_data.job_ranks.size() + 1);
  assert(route_data.breaks_counts.size() == route_data.job_ranks.size() + 1);

  if (vehicle.has_end() && !route_data.job_ranks.empty()) {
    // Update with last route leg.
    assert(previous_index.has_value());
    eval_sum +=
      vehicle.eval(previous_index.value(), vehicle.end.value().index());
  }
  if (!vehicle.ok_for_travel_time(eval_sum.duration)) {
    throw InputException(
      std::format("Route over max_travel_time for vehicle {}.", vehicle.id));
  }
  if (!vehicle.ok_for_distance(eval_sum.distance)) {
    throw InputException(
      std::format("Route over max_distance for vehicle {}.", vehicle.id));
  }

  if (vehicle.max_tasks < route_data.job_ranks.size()) {
    throw InputException(
      std::format("Too many tasks for vehicle {}.", vehicle.id));
  }

  if (!expected_delivery_ranks.empty()) {
    throw InputException(
      std::format("Invalid shipment in route for vehicle {}.", vehicle.id));
  }

  return route_data;
}

void RawRoute::set_route(const Input& input, const std::vector<Index>& r) {
  route = r;
  update_amounts(input);
}

void RawRoute::update_amounts(const Input& input) {
  auto step_size = route.size() + 2;
  _fwd_pickups.resize(route.size());
  _fwd_deliveries.resize(route.size());
  _bwd_deliveries.resize(route.size());
  _bwd_pickups.resize(route.size());
  _pd_loads.resize(route.size());
  _nb_pickups.resize(route.size());
  _nb_deliveries.resize(route.size());

  _current_loads.resize(step_size);
  _fwd_peaks.resize(step_size);
  _bwd_peaks.resize(step_size);

  if (route.empty()) {
    // So that check in is_valid_addition_for_capacity is consistent
    // with empty routes.
    std::ranges::fill(_fwd_peaks, _zero);
    std::ranges::fill(_bwd_peaks, _zero);
    // So that check against break max_load and margins computations
    // are consistent with empty routes.
    std::ranges::fill(_current_loads, _zero);
    return;
  }

  Amount current_pickups(_zero);
  Amount current_deliveries(_zero);
  Amount current_pd_load(_zero);
  unsigned current_nb_pickups = 0;
  unsigned current_nb_deliveries = 0;

  for (std::size_t i = 0; i < route.size(); ++i) {
    switch (const auto& job = input.jobs[route[i]]; job.type) {
      using enum JOB_TYPE;
    case SINGLE:
      current_pickups += job.pickup;
      current_deliveries += job.delivery;
      break;
    case PICKUP:
      current_pd_load += job.pickup;
      current_nb_pickups += 1;
      break;
    case DELIVERY:
      assert(job.delivery <= current_pd_load);
      current_pd_load -= job.delivery;
      current_nb_deliveries += 1;
      break;
    }
    _fwd_pickups[i] = current_pickups;
    _fwd_deliveries[i] = current_deliveries;
    _pd_loads[i] = current_pd_load;
    assert(current_nb_deliveries <= current_nb_pickups);
    _nb_pickups[i] = current_nb_pickups;
    _nb_deliveries[i] = current_nb_deliveries;
  }
  assert(_pd_loads.back() == _zero);

  current_deliveries = _zero;
  current_pickups = _zero;

  _current_loads.back() = _fwd_pickups.back();
  assert(_current_loads.back() <= capacity);

  for (std::size_t i = 0; i < route.size(); ++i) {
    auto bwd_i = route.size() - i - 1;

    _bwd_deliveries[bwd_i] = current_deliveries;
    _bwd_pickups[bwd_i] = current_pickups;
    _current_loads[bwd_i + 1] =
      _fwd_pickups[bwd_i] + _pd_loads[bwd_i] + current_deliveries;
    assert(_current_loads[bwd_i + 1] <= capacity);
    const auto& job = input.jobs[route[bwd_i]];
    if (job.type == JOB_TYPE::SINGLE) {
      current_deliveries += job.delivery;
      current_pickups += job.pickup;
    }
  }
  _current_loads[0] = current_deliveries;
  assert(_current_loads[0] <= capacity);

  auto peak = _current_loads[0];
  _fwd_peaks[0] = peak;
  for (std::size_t s = 1; s < _fwd_peaks.size(); ++s) {
    // Handle max component-wise.
    for (std::size_t r = 0; r < _zero.size(); ++r) {
      peak[r] = std::max(peak[r], _current_loads[s][r]);
    }
    _fwd_peaks[s] = peak;
  }

  peak = _current_loads.back();
  _bwd_peaks.back() = peak;
  for (std::size_t s = 1; s < _bwd_peaks.size(); ++s) {
    auto bwd_s = _bwd_peaks.size() - s - 1;
    // Handle max component-wise.
    for (std::size_t r = 0; r < _zero.size(); ++r) {
      peak[r] = std::max(peak[r], _current_loads[bwd_s][r]);
    }
    _bwd_peaks[bwd_s] = peak;
  }

  if (route.empty()) {
    _delivery_margin = capacity;
    _pickup_margin = capacity;
  } else {
    assert(!_fwd_pickups.empty());
    const auto& pickups_sum = _fwd_pickups.back();

    for (unsigned i = 0; i < _zero.size(); ++i) {
      _delivery_margin[i] = capacity[i] - _current_loads[0][i];
      _pickup_margin[i] = capacity[i] - pickups_sum[i];
    }
  }
}

bool RawRoute::has_pending_delivery_after_rank(const Index rank) const {
  return _nb_deliveries[rank] < _nb_pickups[rank];
}

bool RawRoute::has_delivery_after_rank(const Index rank) const {
  assert(rank < _nb_deliveries.size());
  return _nb_deliveries[rank] < _nb_deliveries.back();
}

bool RawRoute::has_pickup_up_to_rank(const Index rank) const {
  assert(rank < _nb_pickups.size());
  return 0 < _nb_pickups[rank];
}

bool RawRoute::is_valid_addition_for_capacity(const Input&,
                                              const Amount& pickup,
                                              const Amount& delivery,
                                              const Index rank) const {
  assert(rank <= route.size());

  return (_fwd_peaks[rank] + delivery <= capacity) &&
         (_bwd_peaks[rank] + pickup <= capacity);
}

bool RawRoute::is_valid_addition_for_load(const Input&,
                                          const Amount& pickup,
                                          const Index rank) const {
  assert(rank <= route.size());

  const auto& load = route.empty() ? _zero : _current_loads[rank];
  return load + pickup <= capacity;
}

bool RawRoute::is_valid_addition_for_capacity_margins(
  const Input&,
  const Amount& pickup,
  const Amount& delivery,
  const Index first_rank,
  const Index last_rank) const {
  assert(1 <= last_rank);
  assert(last_rank <= route.size() + 1);

  const auto& first_deliveries =
    (first_rank == 0) ? _current_loads[0] : _bwd_deliveries[first_rank - 1];

  const auto& first_pickups =
    (first_rank == 0) ? _zero : _fwd_pickups[first_rank - 1];

  auto replaced_deliveries = first_deliveries - _bwd_deliveries[last_rank - 1];

  return (_fwd_peaks[first_rank] + delivery <=
          capacity + replaced_deliveries) &&
         (_bwd_peaks[last_rank] + pickup <=
          capacity + _fwd_pickups[last_rank - 1] - first_pickups);
}

template <std::forward_iterator Iter>
bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  Amount delivery,
  const Iter first_job,
  const Iter last_job,
  const Index first_rank,
  const Index last_rank) const {
  assert(first_rank <= last_rank);
  assert(last_rank <= route.size() + 1);

  const auto& init_load = (route.empty()) ? _zero : _current_loads[0];

  const auto& first_deliveries =
    (first_rank == 0) ? init_load : _bwd_deliveries[first_rank - 1];

  const auto& last_deliveries =
    (last_rank == 0) ? init_load : _bwd_deliveries[last_rank - 1];

  auto replaced_deliveries = first_deliveries - last_deliveries;

  delivery += ((route.empty()) ? _zero : _current_loads[first_rank]) -
              replaced_deliveries;

  bool valid = (delivery <= capacity);

  for (auto job_iter = first_job; job_iter != last_job; ++job_iter) {
    if (!valid) {
      break;
    }

    auto& job = input.jobs[*job_iter];
    delivery += job.pickup;
    delivery -= job.delivery;

    valid = (delivery <= capacity);
  }

  return valid;
}

const Amount& RawRoute::job_deliveries_sum() const {
  return route.empty() ? _zero : _current_loads[0];
}

const Amount& RawRoute::job_pickups_sum() const {
  return route.empty() ? _zero : _fwd_pickups.back();
}

const Amount& RawRoute::delivery_margin() const {
  return _delivery_margin;
}

const Amount& RawRoute::pickup_margin() const {
  return _pickup_margin;
}

Amount RawRoute::pickup_in_range(Index i, Index j) const {
  assert(i <= j && j <= _fwd_pickups.size());
  if (i == j || route.empty()) {
    return _zero;
  }
  if (i == 0) {
    return _fwd_pickups[j - 1];
  }
  return _fwd_pickups[j - 1] - _fwd_pickups[i - 1];
}

Amount RawRoute::delivery_in_range(Index i, Index j) const {
  assert(i <= j && j <= _bwd_deliveries.size());
  if (i == j || route.empty()) {
    return _zero;
  }
  const auto& before_deliveries =
    (i == 0) ? _current_loads[0] : _bwd_deliveries[i - 1];
  return before_deliveries - _bwd_deliveries[j - 1];
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

template <std::forward_iterator Iter>
void RawRoute::replace(const Input& input,
                       const Iter first_job,
                       const Iter last_job,
                       const Index first_rank,
                       const Index last_rank) {
  assert(first_rank <= last_rank);

  route.erase(route.begin() + first_rank, route.begin() + last_rank);
  route.insert(route.begin() + first_rank, first_job, last_job);

  update_amounts(input);
}

template bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  Amount delivery,
  const std::vector<Index>::iterator first_job,
  const std::vector<Index>::iterator last_job,
  const Index first_rank,
  const Index last_rank) const;
template bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  Amount delivery,
  const std::vector<Index>::const_iterator first_job,
  const std::vector<Index>::const_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;
template bool RawRoute::is_valid_addition_for_capacity_inclusion(
  const Input& input,
  Amount delivery,
  const std::vector<Index>::reverse_iterator first_job,
  const std::vector<Index>::reverse_iterator last_job,
  const Index first_rank,
  const Index last_rank) const;
template void RawRoute::replace(const Input& input,
                                std::vector<Index>::iterator first_job,
                                std::vector<Index>::iterator last_job,
                                const Index first_rank,
                                const Index last_rank);
template void RawRoute::replace(const Input& input,
                                std::vector<Index>::const_iterator first_job,
                                std::vector<Index>::const_iterator last_job,
                                const Index first_rank,
                                const Index last_rank);
} // namespace vroom
