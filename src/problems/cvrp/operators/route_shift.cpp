/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_shift.h"

namespace vroom {
namespace cvrp {

RouteShift::RouteShift(const Input& input,
                       const utils::SolutionState& sol_state,
                       RawRoute& s_route,
                       Index s_vehicle,
                       RawRoute& t_route,
                       Index t_vehicle)
  : Operator(OperatorName::RouteShift,
             input,
             sol_state,
             s_route,
             s_vehicle,
             0, // Dummy value
             t_route,
             t_vehicle,
             0), // Dummy value
    shift_to_start(false),
    shift_to_end(false) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 3);
  assert(!t_route.empty());

  assert(_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] == 0);
}

Eval RouteShift::gain_upper_bound() {
  const auto& s_v = _input.vehicles[s_vehicle];
  const auto& t_v = _input.vehicles[t_vehicle];

  // Source route gain.
  auto first_s_index = _input.jobs[s_route.front()].index();
  if (s_v.has_start()) {
    s_gain += s_v.eval(s_v.start.value().index(), first_s_index);
  }

  auto last_s_index = _input.jobs[s_route.back()].index();
  if (s_v.has_end()) {
    s_gain += s_v.eval(last_s_index, s_v.end.value().index());
  }

  s_gain += _sol_state.fwd_costs[s_vehicle][s_vehicle].back();

  // Target route gain options when inserting source route at start or
  // end or target route.
  if (t_v.has_start()) {
    auto first_t_index = _input.jobs[t_route.front()].index();
    _start_t_gain += t_v.eval(t_v.start.value().index(), first_t_index);
    _start_t_gain -= t_v.eval(t_v.start.value().index(), first_s_index);
    _start_t_gain -= t_v.eval(last_s_index, first_t_index);
  }

  if (t_v.has_end()) {
    auto last_t_index = _input.jobs[t_route.back()].index();
    _end_t_gain += t_v.eval(last_t_index, t_v.end.value().index());
    _end_t_gain -= t_v.eval(last_s_index, t_v.end.value().index());
    _end_t_gain -= t_v.eval(last_t_index, first_s_index);
  }

  _start_t_gain -= _sol_state.fwd_costs[s_vehicle][t_vehicle].back();
  _end_t_gain -= _sol_state.fwd_costs[s_vehicle][t_vehicle].back();

  _gain_upper_bound_computed = true;

  return s_gain + std::max(_start_t_gain, _end_t_gain);
}

void RouteShift::compute_gain() {
  assert(_gain_upper_bound_computed);
  assert(is_start_valid or is_end_valid);

  stored_gain = s_gain;

  if (_start_t_gain < _end_t_gain) {
    if (is_end_valid) {
      shift_to_end = true;
      stored_gain += _end_t_gain;
    } else {
      shift_to_start = true;
      stored_gain += _start_t_gain;
    }
  } else {
    if (is_start_valid) {
      shift_to_start = true;
      stored_gain += _start_t_gain;
    } else {
      shift_to_end = true;
      stored_gain += _end_t_gain;
    }
  }

  gain_computed = true;
}

bool RouteShift::is_valid() {
  assert(_gain_upper_bound_computed);

  const auto& s_delivery = source.job_deliveries_sum();
  const auto& s_pickup = source.job_pickups_sum();

  const auto& t_v = _input.vehicles[t_vehicle];
  const auto t_travel_time = _sol_state.route_evals[t_vehicle].duration;

  is_start_valid =
    target.is_valid_addition_for_capacity(_input, s_pickup, s_delivery, 0);

  if (is_start_valid) {
    is_start_valid =
      (t_travel_time <= t_v.max_travel_time + _start_t_gain.duration) and
      target.is_valid_addition_for_capacity_inclusion(_input,
                                                      s_delivery,
                                                      s_route.begin(),
                                                      s_route.end(),
                                                      0,
                                                      0);
  }

  is_end_valid = target.is_valid_addition_for_capacity(_input,
                                                       s_pickup,
                                                       s_delivery,
                                                       t_route.size());

  if (is_end_valid) {
    is_end_valid =
      (t_travel_time <= t_v.max_travel_time + _end_t_gain.duration) and
      target.is_valid_addition_for_capacity_inclusion(_input,
                                                      s_delivery,
                                                      s_route.begin(),
                                                      s_route.end(),
                                                      t_route.size(),
                                                      t_route.size());
  }

  return (is_start_valid or is_end_valid);
}

void RouteShift::apply() {
  if (shift_to_start) {
    t_route.insert(t_route.begin(), s_route.begin(), s_route.end());
  } else {
    assert(shift_to_end);

    t_route.insert(t_route.end(), s_route.begin(), s_route.end());
  }

  s_route.erase(s_route.begin(), s_route.end());

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> RouteShift::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> RouteShift::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace cvrp
} // namespace vroom
