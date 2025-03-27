/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/or_opt.h"

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
  const auto& s_v = _input.vehicles[s_vehicle];
  const auto& t_v = _input.vehicles[t_vehicle];

  // For source vehicle, we consider the cost of removing edge
  // starting at rank s_rank, already stored in
  // _sol_state.edge_gains[s_vehicle][s_rank].

  // For target vehicle, we consider the cost of adding source edge at
  // rank t_rank. reverse_* checks whether we should change the
  // source edge order.
  const Index s_index = _input.jobs[s_route[s_rank]].index();
  const Index after_s_index = _input.jobs[s_route[s_rank + 1]].index();

  Eval previous_cost;
  Eval next_cost;
  Eval reverse_previous_cost;
  Eval reverse_next_cost;
  Eval old_edge_cost;

  if (t_rank == t_route.size()) {
    if (t_route.empty()) {
      if (t_v.has_start()) {
        previous_cost = t_v.eval(t_v.start.value().index(), s_index);
        reverse_previous_cost =
          t_v.eval(t_v.start.value().index(), after_s_index);
      }
      if (t_v.has_end()) {
        next_cost = t_v.eval(after_s_index, t_v.end.value().index());
        reverse_next_cost = t_v.eval(s_index, t_v.end.value().index());
      }
    } else {
      // Adding edge past the end after a real job.
      auto p_index = _input.jobs[t_route[t_rank - 1]].index();
      previous_cost = t_v.eval(p_index, s_index);
      reverse_previous_cost = t_v.eval(p_index, after_s_index);
      if (t_v.has_end()) {
        auto n_index = t_v.end.value().index();
        old_edge_cost = t_v.eval(p_index, n_index);
        next_cost = t_v.eval(after_s_index, n_index);
        reverse_next_cost = t_v.eval(s_index, n_index);
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = _input.jobs[t_route[t_rank]].index();
    next_cost = t_v.eval(after_s_index, n_index);
    reverse_next_cost = t_v.eval(s_index, n_index);

    if (t_rank == 0) {
      if (t_v.has_start()) {
        auto p_index = t_v.start.value().index();
        previous_cost = t_v.eval(p_index, s_index);
        reverse_previous_cost = t_v.eval(p_index, after_s_index);
        old_edge_cost = t_v.eval(p_index, n_index);
      }
    } else {
      auto p_index = _input.jobs[t_route[t_rank - 1]].index();
      previous_cost = t_v.eval(p_index, s_index);
      reverse_previous_cost = t_v.eval(p_index, after_s_index);
      old_edge_cost = t_v.eval(p_index, n_index);
    }
  }

  // Gain for source vehicle, including cost of moved edge.
  s_gain =
    _sol_state.edge_gains[s_vehicle][s_rank] + s_v.eval(s_index, after_s_index);

  if (s_route.size() == 2) {
    s_gain.cost += s_v.fixed_cost();
  }

  // Gain for target vehicle, including cost of moved edge.
  _normal_t_gain = old_edge_cost - previous_cost - next_cost -
                   t_v.eval(s_index, after_s_index);

  _reversed_t_gain = old_edge_cost - reverse_previous_cost - reverse_next_cost -
                     t_v.eval(after_s_index, s_index);

  if (t_route.empty()) {
    _normal_t_gain.cost -= t_v.fixed_cost();
    _reversed_t_gain.cost -= t_v.fixed_cost();
  }

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
