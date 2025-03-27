/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/two_opt.h"

namespace vroom::cvrp {

TwoOpt::TwoOpt(const Input& input,
               const utils::SolutionState& sol_state,
               RawRoute& s_route,
               Index s_vehicle,
               Index s_rank,
               RawRoute& t_route,
               Index t_vehicle,
               Index t_rank)
  : Operator(OperatorName::TwoOpt,
             input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank),
    _s_delivery(source.bwd_deliveries(s_rank)),
    _t_delivery(target.bwd_deliveries(t_rank)) {
  assert(s_vehicle != t_vehicle);
  assert(!s_route.empty());
  assert(!t_route.empty());
  assert(s_rank < s_route.size());
  assert(t_rank < t_route.size());

  assert(_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] <= s_rank + 1);
  assert(_sol_state.bwd_skill_rank[t_vehicle][s_vehicle] <= t_rank + 1);
}

void TwoOpt::compute_gain() {
  const auto& s_v = _input.vehicles[s_vehicle];
  const auto& t_v = _input.vehicles[t_vehicle];

  const Index s_index = _input.jobs[s_route[s_rank]].index();
  const Index t_index = _input.jobs[t_route[t_rank]].index();
  const Index last_s = _input.jobs[s_route.back()].index();
  const Index last_t = _input.jobs[t_route.back()].index();

  Index new_last_s = last_t;
  Index new_last_t = last_s;

  // Cost of swapping route for vehicle s_vehicle after step
  // s_rank with route for vehicle t_vehicle after step
  // t_rank.

  // Basic costs in case we really swap jobs and not only the end of
  // the route. Otherwise remember that last job does not change.
  if (s_rank < s_route.size() - 1) {
    const Index next_index = _input.jobs[s_route[s_rank + 1]].index();
    s_gain += s_v.eval(s_index, next_index);
    t_gain -= t_v.eval(t_index, next_index);

    // Account for the change in cost across vehicles for the end of
    // source route. Cost of remaining route retrieved by subtracting
    // intermediate cost to overall cost.
    s_gain += _sol_state.fwd_costs[s_vehicle][s_vehicle].back();
    s_gain -= _sol_state.fwd_costs[s_vehicle][s_vehicle][s_rank + 1];
    t_gain -= _sol_state.fwd_costs[s_vehicle][t_vehicle].back();
    t_gain += _sol_state.fwd_costs[s_vehicle][t_vehicle][s_rank + 1];
  } else {
    new_last_t = t_index;
  }
  if (t_rank < t_route.size() - 1) {
    const Index next_index = _input.jobs[t_route[t_rank + 1]].index();
    t_gain += t_v.eval(t_index, next_index);
    s_gain -= s_v.eval(s_index, next_index);

    // Account for the change in cost across vehicles for the end of
    // target route. Cost of remaining route retrieved by subtracting
    // intermediate cost to overall cost.
    t_gain += _sol_state.fwd_costs[t_vehicle][t_vehicle].back();
    t_gain -= _sol_state.fwd_costs[t_vehicle][t_vehicle][t_rank + 1];
    s_gain -= _sol_state.fwd_costs[t_vehicle][s_vehicle].back();
    s_gain += _sol_state.fwd_costs[t_vehicle][s_vehicle][t_rank + 1];
  } else {
    new_last_s = s_index;
  }

  // Handling end route cost change because vehicle ends can be
  // different or none.
  if (s_v.has_end()) {
    auto end_s = s_v.end.value().index();
    s_gain += s_v.eval(last_s, end_s);
    s_gain -= s_v.eval(new_last_s, end_s);
  }
  if (t_v.has_end()) {
    auto end_t = t_v.end.value().index();
    t_gain += t_v.eval(last_t, end_t);
    t_gain -= t_v.eval(new_last_t, end_t);
  }

  stored_gain = s_gain + t_gain;
  gain_computed = true;
}

bool TwoOpt::is_valid() {
  assert(gain_computed);

  const auto& t_pickup = target.bwd_pickups(t_rank);

  const auto& s_pickup = source.bwd_pickups(s_rank);

  return is_valid_for_source_range_bounds() &&
         is_valid_for_target_range_bounds() &&
         source.is_valid_addition_for_capacity_margins(_input,
                                                       t_pickup,
                                                       _t_delivery,
                                                       s_rank + 1,
                                                       s_route.size()) &&
         target.is_valid_addition_for_capacity_margins(_input,
                                                       s_pickup,
                                                       _s_delivery,
                                                       t_rank + 1,
                                                       t_route.size()) &&
         source.is_valid_addition_for_capacity_inclusion(_input,
                                                         _t_delivery,
                                                         t_route.begin() +
                                                           t_rank + 1,
                                                         t_route.end(),
                                                         s_rank + 1,
                                                         s_route.size()) &&
         target.is_valid_addition_for_capacity_inclusion(_input,
                                                         _s_delivery,
                                                         s_route.begin() +
                                                           s_rank + 1,
                                                         s_route.end(),
                                                         t_rank + 1,
                                                         t_route.size());
}

void TwoOpt::apply() {
  auto nb_source = s_route.size() - 1 - s_rank;

  t_route.insert(t_route.begin() + t_rank + 1,
                 s_route.begin() + s_rank + 1,
                 s_route.end());
  s_route.erase(s_route.begin() + s_rank + 1, s_route.end());
  s_route.insert(s_route.end(),
                 t_route.begin() + t_rank + 1 + nb_source,
                 t_route.end());
  t_route.erase(t_route.begin() + t_rank + 1 + nb_source, t_route.end());

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> TwoOpt::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> TwoOpt::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
