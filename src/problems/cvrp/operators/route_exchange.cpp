/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_exchange.h"
#include "utils/helpers.h"

namespace vroom::cvrp {

RouteExchange::RouteExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_route,
                             Index s_vehicle,
                             RawRoute& t_route,
                             Index t_vehicle)
  : Operator(OperatorName::RouteExchange,
             input,
             sol_state,
             s_route,
             s_vehicle,
             0, // Dummy value
             t_route,
             t_vehicle,
             0) { // Dummy value
  assert(s_vehicle != t_vehicle);
  assert(!s_route.empty() || !t_route.empty());

  // Whole routes should be transferable.
  assert(_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] == 0);
  assert(_sol_state.bwd_skill_rank[t_vehicle][s_vehicle] == 0);
}

void RouteExchange::compute_gain() {
  s_gain = t_route.empty()
             ? _sol_state.route_evals[s_vehicle]
             : std::get<0>(utils::addition_eval_delta(_input,
                                                      _sol_state,
                                                      source,
                                                      0,
                                                      s_route.size(),
                                                      target,
                                                      0,
                                                      t_route.size()));

  t_gain = s_route.empty()
             // The upstream empty-source branch reads
             // route_evals[s_vehicle] (always 0 here), so the removal
             // gain of the emptied target route is never accounted.
             // That orientation cannot reach positive stored_gain
             // upstream and is thus never applied (its debug
             // accounting assert would fail). Preference promotion is
             // the first user of this orientation, so there the empty
             // source's counterpart uses the target's route eval,
             // which is the mirror of the s_gain branch. Inputs
             // without soft-skills keep the exact upstream value
             // (zero change invariant).
             ? (has_soft_skills() ? _sol_state.route_evals[t_vehicle]
                                  : _sol_state.route_evals[s_vehicle])
             : std::get<0>(utils::addition_eval_delta(_input,
                                                      _sol_state,
                                                      target,
                                                      0,
                                                      t_route.size(),
                                                      source,
                                                      0,
                                                      s_route.size()));

  stored_gain = s_gain + t_gain;

  if (has_soft_skills() && has_preference_context()) {
    // Whole routes are exchanged: sum the delta of every task moved
    // to the other vehicle. After the swap a task keeps its route
    // content and ranks (only the vehicle start/end change), so its
    // placement eval is the marginal addition on its own route; the
    // destination rank is always valid there. Shipment preference
    // counts once through its PICKUP endpoint (DELIVERY endpoints
    // contribute nothing); evals use the frozen pre-move state (Q6).
    int delta = 0;
    for (Index r = 0; r < s_route.size(); ++r) {
      delta += task_move_delta(s_route[r], s_vehicle, t_vehicle, source, r);
    }
    for (Index r = 0; r < t_route.size(); ++r) {
      delta += task_move_delta(t_route[r], t_vehicle, s_vehicle, target, r);
    }
    preference_delta = delta;
  }

  gain_computed = true;
}

bool RouteExchange::is_valid() {
  assert(gain_computed);

  return is_valid_for_source_range_bounds() &&
         is_valid_for_target_range_bounds() &&
         (source.max_load() <= _input.vehicles[t_vehicle].capacity) &&
         (target.max_load() <= _input.vehicles[s_vehicle].capacity);
}

void RouteExchange::apply() {
  std::swap(s_route, t_route);

  source.update_amounts(_input);
  target.update_amounts(_input);
}

std::vector<Index> RouteExchange::addition_candidates() const {
  return {s_vehicle, t_vehicle};
}

std::vector<Index> RouteExchange::update_candidates() const {
  return {s_vehicle, t_vehicle};
}

} // namespace vroom::cvrp
