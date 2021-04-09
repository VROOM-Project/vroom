/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/route_exchange.h"

namespace vroom {
namespace cvrp {

RouteExchange::RouteExchange(const Input& input,
                             const utils::SolutionState& sol_state,
                             RawRoute& s_route,
                             Index s_vehicle,
                             RawRoute& t_route,
                             Index t_vehicle)
  : Operator(input,
             sol_state,
             s_route,
             s_vehicle,
             0, // Dummy value
             t_route,
             t_vehicle,
             0) { // Dummy value
  assert(s_vehicle != t_vehicle);
  assert(s_route.size() >= 1 or t_route.size() >= 1);

  // Whole routes should be transferable.
  assert(_sol_state.bwd_skill_rank[s_vehicle][t_vehicle] == 0);
  assert(_sol_state.bwd_skill_rank[t_vehicle][s_vehicle] == 0);
}

void RouteExchange::compute_gain() {
  const auto& s_v = _input.vehicles[s_vehicle];
  const auto& t_v = _input.vehicles[t_vehicle];

  Gain new_cost = 0;
  Gain previous_cost = 0;

  if (s_route.size() > 0) {
    // Handle changes at route start.
    auto first_s_index = _input.jobs[s_route.front()].index();
    if (s_v.has_start()) {
      previous_cost += s_v.cost(s_v.start.value().index(), first_s_index);
    }
    if (t_v.has_start()) {
      new_cost += t_v.cost(t_v.start.value().index(), first_s_index);
    }

    // Handle changes at route end.
    auto last_s_index = _input.jobs[s_route.back()].index();
    if (s_v.has_end()) {
      previous_cost += s_v.cost(last_s_index, s_v.end.value().index());
    }
    if (t_v.has_end()) {
      new_cost += t_v.cost(last_s_index, t_v.end.value().index());
    }

    // Handle inner cost change for route.
    previous_cost += _sol_state.fwd_costs[s_vehicle][s_vehicle].back();
    new_cost += _sol_state.fwd_costs[s_vehicle][t_vehicle].back();
  }

  if (t_route.size() > 0) {
    // Handle changes at route start.
    auto first_t_index = _input.jobs[t_route.front()].index();
    if (t_v.has_start()) {
      previous_cost += t_v.cost(t_v.start.value().index(), first_t_index);
    }
    if (s_v.has_start()) {
      new_cost += s_v.cost(s_v.start.value().index(), first_t_index);
    }

    // Handle changes at route end.
    auto last_t_index = _input.jobs[t_route.back()].index();
    if (t_v.has_end()) {
      previous_cost += t_v.cost(last_t_index, t_v.end.value().index());
    }
    if (s_v.has_end()) {
      new_cost += s_v.cost(last_t_index, s_v.end.value().index());
    }

    // Handle inner cost change for route.
    previous_cost += _sol_state.fwd_costs[t_vehicle][t_vehicle].back();
    new_cost += _sol_state.fwd_costs[t_vehicle][s_vehicle].back();
  }

  stored_gain = previous_cost - new_cost;
  gain_computed = true;
}

bool RouteExchange::is_valid() {
  return (source.max_load() <= _input.vehicles[t_vehicle].capacity) &&
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

} // namespace cvrp
} // namespace vroom
