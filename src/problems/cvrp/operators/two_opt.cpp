/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/two_opt.h"
#include "utils/helpers.h"

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
  // Moving at least a portion with more than 3 jobs, else the
  // situation is covered by other moves.
  assert(s_rank + 3u < s_route.size() || t_rank + 3u < t_route.size());

  assert(_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] <= s_rank + 1);
  assert(_sol_state.bwd_skill_rank[t_vehicle][s_vehicle] <= t_rank + 1);
}

void TwoOpt::compute_gain() {
  s_gain = (t_rank + 1u < t_route.size())
             ? std::get<0>(utils::addition_eval_delta(_input,
                                                      _sol_state,
                                                      source,
                                                      s_rank + 1,
                                                      s_route.size(),
                                                      target,
                                                      t_rank + 1,
                                                      t_route.size()))
             : utils::removal_gain(_input,
                                   _sol_state,
                                   source,
                                   s_rank + 1,
                                   s_route.size());

  t_gain = (s_rank + 1u < s_route.size())
             ? std::get<0>(utils::addition_eval_delta(_input,
                                                      _sol_state,
                                                      target,
                                                      t_rank + 1,
                                                      t_route.size(),
                                                      source,
                                                      s_rank + 1,
                                                      s_route.size()))
             : utils::removal_gain(_input,
                                   _sol_state,
                                   target,
                                   t_rank + 1,
                                   t_route.size());

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
