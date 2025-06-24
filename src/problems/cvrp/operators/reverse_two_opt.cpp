/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/reverse_two_opt.h"

namespace vroom::cvrp {

ReverseTwoOpt::ReverseTwoOpt(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_route,
                             Index s_vehicle,
                             Index s_rank,
                             RawRoute& t_route,
                             Index t_vehicle,
                             Index t_rank)
  : Operator(OperatorName::ReverseTwoOpt,
             input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank),
    _s_delivery(source.bwd_deliveries(s_rank)),
    _t_delivery(target.fwd_deliveries(t_rank)) {
  assert(s_vehicle != t_vehicle);
  assert(!s_route.empty());
  assert(!t_route.empty());
  assert(s_rank < s_route.size());
  assert(t_rank < t_route.size());

  assert(_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] <= s_rank + 1);
  assert(t_rank < _sol_state.fwd_skill_rank[t_vehicle][s_vehicle]);
}

void ReverseTwoOpt::compute_gain() {
  const auto& s_v = _input.vehicles[s_vehicle];
  const auto& t_v = _input.vehicles[t_vehicle];

  const Index s_index = _input.jobs[s_route[s_rank]].index();
  const Index t_index = _input.jobs[t_route[t_rank]].index();
  const Index last_s = _input.jobs[s_route.back()].index();
  const Index first_t = _input.jobs[t_route.front()].index();

  const bool last_in_source = (s_rank == s_route.size() - 1);
  const bool last_in_target = (t_rank == t_route.size() - 1);

  // Cost of swapping route for vehicle s_vehicle after step
  // s_rank with route for vehicle t_vehicle up to step
  // t_rank, but reversed.

  // Add new source -> target edge.
  s_gain -= s_v.eval(s_index, t_index);

  // Cost of reversing target route portion. First remove forward cost
  // for beginning of target route as seen from target vehicle. Then
  // add backward cost for beginning of target route as seen from
  // source vehicle since it's the new source route end.
  t_gain += _sol_state.fwd_costs[t_vehicle][t_vehicle][t_rank];
  s_gain -= _sol_state.bwd_costs[t_vehicle][s_vehicle][t_rank];

  if (last_in_target) {
    // Spare cost to target route end if any.
    if (t_v.has_end()) {
      const auto end_t = t_v.end.value().index();
      t_gain += t_v.eval(t_index, end_t);
    }
  } else {
    // Spare next edge in target route.
    const Index next_index = _input.jobs[t_route[t_rank + 1]].index();
    t_gain += t_v.eval(t_index, next_index);
  }

  if (!last_in_source) {
    // Spare next edge in source route.
    const Index next_index = _input.jobs[s_route[s_rank + 1]].index();
    s_gain += s_v.eval(s_index, next_index);

    // Part of source route is moved to target route.
    const Index next_s_index = _input.jobs[s_route[s_rank + 1]].index();

    // Cost or reverting source route portion. First remove forward
    // cost for end of source route as seen from source vehicle
    // (subtracting intermediate cost to overall cost). Then add
    // backward cost for end of source route as seen from target
    // vehicle since it's the new target route start.
    s_gain += _sol_state.fwd_costs[s_vehicle][s_vehicle].back();
    s_gain -= _sol_state.fwd_costs[s_vehicle][s_vehicle][s_rank + 1];
    t_gain -= _sol_state.bwd_costs[s_vehicle][t_vehicle].back();
    t_gain += _sol_state.bwd_costs[s_vehicle][t_vehicle][s_rank + 1];

    if (last_in_target) {
      if (t_v.has_end()) {
        // Handle target route new end.
        const auto end_t = t_v.end.value().index();
        t_gain -= t_v.eval(next_s_index, end_t);
      }
    } else {
      // Add new target -> source edge.
      const Index next_t_index = _input.jobs[t_route[t_rank + 1]].index();
      t_gain -= t_v.eval(next_s_index, next_t_index);
    }
  }

  if (s_v.has_end()) {
    // Update cost to source end because last job changed.
    auto end_s = s_v.end.value().index();
    s_gain += s_v.eval(last_s, end_s);
    s_gain -= s_v.eval(first_t, end_s);
  }

  if (t_v.has_start()) {
    // Spare cost from target start because first job changed.
    auto start_t = t_v.start.value().index();
    t_gain += t_v.eval(start_t, first_t);
    if (!last_in_source) {
      t_gain -= t_v.eval(start_t, last_s);
    } else {
      // No job from source route actually swapped to target route.
      if (!last_in_target) {
        // Going straight from start to next job in target route.
        const Index next_index = _input.jobs[t_route[t_rank + 1]].index();
        t_gain -= t_v.eval(start_t, next_index);
      }
    }
  }

  if (last_in_source && last_in_target) {
    // Emptying target route.
    t_gain.cost += t_v.fixed_cost();
  }

  stored_gain = s_gain + t_gain;
  gain_computed = true;
}

bool ReverseTwoOpt::is_valid() {
  assert(gain_computed);

  const auto& t_pickup = target.fwd_pickups(t_rank);

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
                                                       0,
                                                       t_rank + 1) &&
         source.is_valid_addition_for_capacity_inclusion(_input,
                                                         _t_delivery,
                                                         t_route.rbegin() +
                                                           t_route.size() - 1 -
                                                           t_rank,
                                                         t_route.rend(),
                                                         s_rank + 1,
                                                         s_route.size()) &&
         target.is_valid_addition_for_capacity_inclusion(_input,
                                                         _s_delivery,
                                                         s_route.rbegin(),
                                                         s_route.rbegin() +
                                                           s_route.size() - 1 -
                                                           s_rank,
                                                         0,
                                                         t_rank + 1);
}

void ReverseTwoOpt::apply() {
  auto nb_source = s_route.size() - 1 - s_rank;

  t_route.insert(t_route.begin(),
                 s_route.rbegin(),
                 s_route.rbegin() + nb_source);
  s_route.erase(s_route.begin() + s_rank + 1, s_route.end());
  s_route.insert(s_route.end(),
                 t_route.rend() - t_rank - nb_source - 1,
                 t_route.rend() - nb_source);
  t_route.erase(t_route.begin() + nb_source,
                t_route.begin() + nb_source + t_rank + 1);

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> ReverseTwoOpt::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> ReverseTwoOpt::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
