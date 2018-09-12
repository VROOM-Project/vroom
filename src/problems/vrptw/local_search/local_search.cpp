/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream> // TODO remove

#include "problems/ls_operator.h"
#include "problems/vrptw/local_search/local_search.h"
#include "problems/vrptw/local_search/relocate.h"
#include "utils/helpers.h"

vrptw_local_search::vrptw_local_search(const input& input, tw_solution& tw_sol)
  : local_search(input, to_raw_solution(tw_sol)), _tw_sol(tw_sol) {
}

void vrptw_local_search::run() {
  std::vector<std::vector<std::unique_ptr<ls_operator>>> best_ops(V);
  for (std::size_t v = 0; v < V; ++v) {
    best_ops[v] = std::vector<std::unique_ptr<ls_operator>>(V);
  }

  // List of source/target pairs we need to test (all at first).
  std::vector<std::pair<index_t, index_t>> s_t_pairs;
  for (unsigned s_v = 0; s_v < V; ++s_v) {
    for (unsigned t_v = 0; t_v < V; ++t_v) {
      if (s_v == t_v) {
        continue;
      }
      s_t_pairs.emplace_back(s_v, t_v);
    }
  }

  std::vector<std::vector<gain_t>> best_gains(V, std::vector<gain_t>(V, 0));

  gain_t best_gain = 1;

  while (best_gain > 0) {
    // Relocate stuff
    for (const auto& s_t : s_t_pairs) {
      if (_sol[s_t.first].size() == 0 or
          !(_sol_state.total_amount(s_t.second) + _amount_lower_bound <=
            _input._vehicles[s_t.second].capacity)) {
        // Don't try to put things in a full vehicle or from an empty
        // vehicle.
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        if (_sol_state.node_gains[s_t.first][s_rank] <=
            best_gains[s_t.first][s_t.second]) {
          // Except if addition cost in route s_t.second is negative
          // (!!), overall gain can't exceed current known best gain.
          continue;
        }
        for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
          vrptw_relocate r(_input,
                           _tw_sol,
                           _sol,
                           _sol_state,
                           s_t.first,
                           s_rank,
                           s_t.second,
                           t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<vrptw_relocate>(r);
          }
        }
      }
    }

    // Find best overall gain.
    best_gain = 0;
    index_t best_source = 0;
    index_t best_target = 0;

    for (unsigned s_v = 0; s_v < V; ++s_v) {
      for (unsigned t_v = 0; t_v < V; ++t_v) {
        if (s_v == t_v) {
          continue;
        }
        if (best_gains[s_v][t_v] > best_gain) {
          best_gain = best_gains[s_v][t_v];
          best_source = s_v;
          best_target = t_v;
        }
      }
    }

    // Apply matching operator.
    if (best_gain > 0) {
      std::cout << best_gain << std::endl;
      exit(0);

      assert(best_ops[best_source][best_target] != nullptr);

      best_ops[best_source][best_target]->apply();

      // Update route costs.
      auto previous_cost = _sol_state.route_costs[best_source] +
                           _sol_state.route_costs[best_target];
      _sol_state.update_route_cost(_sol, best_source);
      _sol_state.update_route_cost(_sol, best_target);
      auto new_cost = _sol_state.route_costs[best_source] +
                      _sol_state.route_costs[best_target];
      assert(new_cost + best_gain == previous_cost);

      _sol_state.update_amounts(_sol, best_source);
      _sol_state.update_amounts(_sol, best_target);

      // Only required for 2-opt* and reverse 2-opt*
      // _sol_state.update_costs(_sol, best_source);
      // _sol_state.update_costs(_sol, best_target);
      // _sol_state.update_skills(_sol, best_source);
      // _sol_state.update_skills(_sol, best_target);

      // Update candidates.
      _sol_state.set_node_gains(_sol, best_source);
      _sol_state.set_node_gains(_sol, best_target);
      _sol_state.set_edge_gains(_sol, best_source);
      _sol_state.set_edge_gains(_sol, best_target);

      // Set gains to zero for what needs to be recomputed in the next
      // round.
      s_t_pairs.clear();
      best_gains[best_source] = std::vector<gain_t>(V, 0);
      best_gains[best_target] = std::vector<gain_t>(V, 0);

      s_t_pairs.emplace_back(best_source, best_target);
      s_t_pairs.emplace_back(best_target, best_source);

      for (unsigned v = 0; v < V; ++v) {
        if (v == best_source or v == best_target) {
          continue;
        }
        s_t_pairs.emplace_back(best_source, v);
        s_t_pairs.emplace_back(v, best_source);
        best_gains[v][best_source] = 0;
        best_gains[best_source][v] = 0;

        s_t_pairs.emplace_back(best_target, v);
        s_t_pairs.emplace_back(v, best_target);
        best_gains[v][best_target] = 0;
        best_gains[best_target][v] = 0;
      }
    }
  }
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
