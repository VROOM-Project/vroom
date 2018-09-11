/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/local_search/local_search.h"
#include "utils/helpers.h"

vrptw_local_search::vrptw_local_search(const input& input, tw_solution& tw_sol)
  : local_search(input, to_raw_solution(tw_sol)), _tw_sol(tw_sol) {
}

void vrptw_local_search::run() {
}

solution_indicators vrptw_local_search::indicators() const {
  solution_indicators si;

  si.unassigned = _sol_state.unassigned.size();
  si.cost = 0;
  for (std::size_t v = 0; v < V; ++v) {
    si.cost += _sol_state.route_costs[v];
  }
  si.used_vehicles = std::count_if(_sol.begin(), _sol.end(), [](const auto& r) {
    return !r.empty();
  });
  return si;
}
