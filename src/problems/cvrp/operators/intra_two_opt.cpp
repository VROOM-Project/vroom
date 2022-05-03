/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_two_opt.h"

namespace vroom {
namespace cvrp {

IntraTwoOpt::IntraTwoOpt(const Input& input,
                         const utils::SolutionState& sol_state,
                         RawRoute& s_route,
                         Index s_vehicle,
                         Index s_rank,
                         Index t_rank)
  : Operator(OperatorName::IntraTwoOpt,
             input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             s_route,
             s_vehicle,
             t_rank) {
  // Assume s_rank < t_rank for symmetry reasons. Set aside cases
  // where t_rank = s_rank + 1, as the move is also an intra_relocate.
  assert(s_route.size() >= 3);
  assert(s_rank < t_rank - 1);
  assert(t_rank < s_route.size());
}

void IntraTwoOpt::compute_gain() {
  const auto& s_v = _input.vehicles[s_vehicle];

  Index s_index = _input.jobs[s_route[s_rank]].index();
  Index t_index = _input.jobs[t_route[t_rank]].index();
  stored_gain = 0;

  // Cost of reversing vehicle route between s_rank and t_rank
  // included.
  stored_gain += _sol_state.fwd_costs[s_vehicle][s_vehicle][t_rank];
  stored_gain -= _sol_state.fwd_costs[s_vehicle][s_vehicle][s_rank];
  stored_gain += _sol_state.bwd_costs[s_vehicle][s_vehicle][s_rank];
  stored_gain -= _sol_state.bwd_costs[s_vehicle][s_vehicle][t_rank];

  // Cost of going to t_rank first instead of s_rank.
  if (s_rank > 0) {
    Index previous_index = _input.jobs[s_route[s_rank - 1]].index();
    stored_gain += s_v.cost(previous_index, s_index);
    stored_gain -= s_v.cost(previous_index, t_index);
  } else {
    if (s_v.has_start()) {
      Index start_index = s_v.start.value().index();
      stored_gain += s_v.cost(start_index, s_index);
      stored_gain -= s_v.cost(start_index, t_index);
    }
  }

  // Cost of going from s_rank after instead of t_rank.
  if (t_rank < s_route.size() - 1) {
    Index next_index = _input.jobs[s_route[t_rank + 1]].index();
    stored_gain += s_v.cost(t_index, next_index);
    stored_gain -= s_v.cost(s_index, next_index);
  } else {
    if (s_v.has_end()) {
      Index end_index = s_v.end.value().index();
      stored_gain += s_v.cost(t_index, end_index);
      stored_gain -= s_v.cost(s_index, end_index);
    }
  }

  gain_computed = true;
}

bool IntraTwoOpt::is_valid() {
  auto rev_t = s_route.rbegin() + (s_route.size() - t_rank - 1);
  auto rev_s_next = s_route.rbegin() + (s_route.size() - s_rank);

  return source
    .is_valid_addition_for_capacity_inclusion(_input,
                                              source.delivery_in_range(s_rank,
                                                                       t_rank +
                                                                         1),
                                              rev_t,
                                              rev_s_next,
                                              s_rank,
                                              t_rank + 1);
}

void IntraTwoOpt::apply() {
  // TODO
}

std::vector<Index> IntraTwoOpt::addition_candidates() const {
  return {};
}

std::vector<Index> IntraTwoOpt::update_candidates() const {
  return {s_vehicle};
}

} // namespace cvrp
} // namespace vroom
