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
             ? _sol_state.route_evals[s_vehicle]
             : std::get<0>(utils::addition_eval_delta(_input,
                                                      _sol_state,
                                                      target,
                                                      0,
                                                      t_route.size(),
                                                      source,
                                                      0,
                                                      s_route.size()));

  stored_gain = s_gain + t_gain;
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
