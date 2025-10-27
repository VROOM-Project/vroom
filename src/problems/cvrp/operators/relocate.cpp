/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/relocate.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

Relocate::Relocate(const Input& input,
                   const utils::SolutionState& sol_state,
                   RawRoute& s_route,
                   Index s_vehicle,
                   Index s_rank,
                   RawRoute& t_route,
                   Index t_vehicle,
                   Index t_rank)
  : Operator(OperatorName::Relocate,
             input,
             sol_state,
             s_route,
             s_vehicle,
             s_rank,
             t_route,
             t_vehicle,
             t_rank) {
  assert(s_vehicle != t_vehicle);
  assert(!s_route.empty());
  assert(s_rank < s_route.size());
  assert(t_rank <= t_route.size());

  assert(_input.vehicle_ok_with_job(t_vehicle, this->s_route[s_rank]));
}

void Relocate::compute_gain() {
  // For source vehicle, we consider the cost of removing job at rank
  // s_rank, already stored.
  s_gain = _sol_state.node_gains[s_vehicle][s_rank];

  if (s_route.size() == 1) {
    s_gain.cost += _input.vehicles[s_vehicle].fixed_cost();
  }

  // For target vehicle, we consider the cost of adding source job at
  // rank t_rank.
  const auto& t_v = _input.vehicles[t_vehicle];
  t_gain = -utils::addition_eval(_input, s_route[s_rank], t_v, t_route, t_rank);

  if (t_route.empty()) {
    t_gain.cost -= t_v.fixed_cost();
  }

  stored_gain = s_gain + t_gain;
  gain_computed = true;
}

bool Relocate::is_valid() {
  assert(gain_computed);
  return is_valid_for_source_range_bounds() &&
         is_valid_for_target_range_bounds() &&
         target
           .is_valid_addition_for_capacity(_input,
                                           _input.jobs[s_route[s_rank]].pickup,
                                           _input.jobs[s_route[s_rank]]
                                             .delivery,
                                           t_rank);
}

void Relocate::apply() {
  auto relocate_job_rank = s_route[s_rank];
  s_route.erase(s_route.begin() + s_rank);
  t_route.insert(t_route.begin() + t_rank, relocate_job_rank);

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> Relocate::addition_candidates() const {
  return {s_vehicle};
}

std::vector<Index> Relocate::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
