/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/or_opt.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

OrOpt::OrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             RawRoute& s_route,
             Index s_vehicle,
             Index s_rank,
             RawRoute& t_route,
             Index t_vehicle,
             Index t_rank)
  : Operator(OperatorName::OrOpt,
             input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank),
    edge_delivery(_input.jobs[this->s_route[s_rank]].delivery +
                  _input.jobs[this->s_route[s_rank + 1]].delivery) {
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 2);
  assert(s_rank < s_route.size() - 1);
  assert(t_rank <= t_route.size());

  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank]));
  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank + 1]));
}

Eval OrOpt::gain_upper_bound() {
  s_gain = utils::removal_gain(_input, _sol_state, source, s_rank, s_rank + 2);

  std::tie(_normal_t_gain, _reversed_t_gain) =
    utils::addition_eval_delta(_input,
                               _sol_state,
                               target,
                               t_rank,
                               t_rank,
                               source,
                               s_rank,
                               s_rank + 2);

  _gain_upper_bound_computed = true;

  return s_gain + std::max(_normal_t_gain, _reversed_t_gain);
}

void OrOpt::compute_gain() {
  assert(_gain_upper_bound_computed);
  assert(is_normal_valid || is_reverse_valid);

  stored_gain = s_gain;

  if (_normal_t_gain < _reversed_t_gain) {
    // Biggest potential gain is obtained when reversing edge.
    if (is_reverse_valid) {
      reverse_s_edge = true;
      stored_gain += _reversed_t_gain;
    } else {
      stored_gain += _normal_t_gain;
    }
  } else {
    // Biggest potential gain is obtained when not reversing edge.
    if (is_normal_valid) {
      stored_gain += _normal_t_gain;
    } else {
      reverse_s_edge = true;
      stored_gain += _reversed_t_gain;
    }
  }

  gain_computed = true;
}

bool OrOpt::is_valid() {
  assert(_gain_upper_bound_computed);

  auto edge_pickup = _input.jobs[s_route[s_rank]].pickup +
                     _input.jobs[s_route[s_rank + 1]].pickup;

  bool valid = is_valid_for_source_range_bounds() &&
               target.is_valid_addition_for_capacity(_input,
                                                     edge_pickup,
                                                     edge_delivery,
                                                     t_rank);

  if (valid) {
    // Keep edge direction.
    auto s_start = s_route.begin() + s_rank;

    const auto& t_v = _input.vehicles[t_vehicle];
    const auto t_eval = _sol_state.route_evals[t_vehicle];

    is_normal_valid =
      t_v.ok_for_range_bounds(t_eval - _normal_t_gain) &&
      target.is_valid_addition_for_capacity_inclusion(_input,
                                                      edge_delivery,
                                                      s_start,
                                                      s_start + 2,
                                                      t_rank,
                                                      t_rank);

    // Reverse edge direction.
    auto s_reverse_start = s_route.rbegin() + s_route.size() - 2 - s_rank;
    is_reverse_valid =
      t_v.ok_for_range_bounds(t_eval - _reversed_t_gain) &&
      target.is_valid_addition_for_capacity_inclusion(_input,
                                                      edge_delivery,
                                                      s_reverse_start,
                                                      s_reverse_start + 2,
                                                      t_rank,
                                                      t_rank);

    valid = (is_normal_valid || is_reverse_valid);
  }

  return valid;
}

void OrOpt::apply() {
  t_route.insert(t_route.begin() + t_rank,
                 s_route.begin() + s_rank,
                 s_route.begin() + s_rank + 2);
  if (reverse_s_edge) {
    std::swap(t_route[t_rank], t_route[t_rank + 1]);
  }

  s_route.erase(s_route.begin() + s_rank, s_route.begin() + s_rank + 2);

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> OrOpt::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> OrOpt::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
