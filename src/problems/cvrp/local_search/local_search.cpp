/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>

#include "problems/cvrp/local_search/2_opt.h"
#include "problems/cvrp/local_search/cross_exchange.h"
#include "problems/cvrp/local_search/exchange.h"
#include "problems/cvrp/local_search/local_search.h"
#include "problems/cvrp/local_search/or_opt.h"
#include "problems/cvrp/local_search/relocate.h"
#include "problems/cvrp/local_search/reverse_2_opt.h"
#include "problems/tsp/tsp.h"
#include "utils/helpers.h"

cvrp_local_search::cvrp_local_search(const input& input,
                                     raw_solution& sol,
                                     unsigned max_nb_jobs_removal)
  : local_search(input, sol),
    _max_nb_jobs_removal(max_nb_jobs_removal),
    _all_routes(V),
    _target_sol(sol),
    _best_sol(sol) {
  // Initialize all route indices.
  std::iota(_all_routes.begin(), _all_routes.end(), 0);

  _best_unassigned = _sol_state.unassigned.size();
  _best_cost = 0;
  for (std::size_t v = 0; v < _sol.size(); ++v) {
    _best_cost += _sol_state.route_cost_for_vehicle(v, _sol[v]);
  }
}

void cvrp_local_search::try_job_additions(const std::vector<index_t>& routes,
                                          double regret_coeff) {
  bool job_added;

  do {
    double best_cost = std::numeric_limits<double>::max();
    index_t best_job = 0;
    index_t best_route = 0;
    index_t best_rank = 0;

    for (const auto j : _sol_state.unassigned) {
      auto& current_amount = _input._jobs[j].amount;
      std::vector<gain_t> best_costs(routes.size(),
                                     std::numeric_limits<gain_t>::max());
      std::vector<index_t> best_ranks(routes.size());

      for (std::size_t i = 0; i < routes.size(); ++i) {
        auto v = routes[i];
        const auto& v_target = _input._vehicles[v];
        const amount_t& v_amount = _sol_state.total_amount(v);

        if (_input.vehicle_ok_with_job(v, j) and
            v_amount + current_amount <= _input._vehicles[v].capacity) {
          for (std::size_t r = 0; r <= _sol[v].size(); ++r) {
            gain_t current_cost =
              addition_cost(_input, _m, j, v_target, _sol[v], r);

            if (current_cost < best_costs[i]) {
              best_costs[i] = current_cost;
              best_ranks[i] = r;
            }
          }
        }
      }

      auto smallest = std::numeric_limits<gain_t>::max();
      auto second_smallest = std::numeric_limits<gain_t>::max();
      std::size_t smallest_idx = std::numeric_limits<gain_t>::max();

      for (std::size_t i = 0; i < routes.size(); ++i) {
        if (best_costs[i] < smallest) {
          smallest_idx = i;
          second_smallest = smallest;
          smallest = best_costs[i];
        } else if (best_costs[i] < second_smallest) {
          second_smallest = best_costs[i];
        }
      }

      // Find best route for current job based on cost of addition and
      // regret cost of not adding.
      for (std::size_t i = 0; i < routes.size(); ++i) {
        auto addition_cost = best_costs[i];
        if (addition_cost == std::numeric_limits<gain_t>::max()) {
          continue;
        }
        auto regret_cost = std::numeric_limits<gain_t>::max();
        if (i == smallest_idx) {
          regret_cost = second_smallest;
        } else {
          regret_cost = smallest;
        }

        double eval = static_cast<double>(addition_cost) -
                      regret_coeff * static_cast<double>(regret_cost);

        if (eval < best_cost) {
          best_cost = eval;
          best_job = j;
          best_route = routes[i];
          best_rank = best_ranks[i];
        }
      }
    }

    job_added = (best_cost < std::numeric_limits<double>::max());

    if (job_added) {
      _sol[best_route].insert(_sol[best_route].begin() + best_rank, best_job);

      // Update amounts after addition.
      const auto& job_amount = _input._jobs[best_job].amount;
      auto& best_fwd_amounts = _sol_state.fwd_amounts[best_route];
      auto previous_cumul = (best_rank == 0) ? amount_t(_input.amount_size())
                                             : best_fwd_amounts[best_rank - 1];
      best_fwd_amounts.insert(best_fwd_amounts.begin() + best_rank,
                              previous_cumul + job_amount);
      std::for_each(best_fwd_amounts.begin() + best_rank + 1,
                    best_fwd_amounts.end(),
                    [&](auto& a) { a += job_amount; });

      auto& best_bwd_amounts = _sol_state.bwd_amounts[best_route];
      best_bwd_amounts.insert(best_bwd_amounts.begin() + best_rank,
                              amount_t(_input.amount_size())); // dummy init
      for (std::size_t i = 0; i <= best_rank; ++i) {
        auto total_amount = _sol_state.fwd_amounts[best_route].back();
        _sol_state.bwd_amounts[best_route][i] =
          total_amount - _sol_state.fwd_amounts[best_route][i];
      }

      // Update cost after addition.
      _sol_state.update_route_cost(_sol, best_route);

      _sol_state.unassigned.erase(best_job);
    }
  } while (job_added);
}

void cvrp_local_search::run_ls_step() {
  std::vector<std::vector<std::unique_ptr<cvrp_ls_operator>>> best_ops(V);
  for (std::size_t v = 0; v < V; ++v) {
    best_ops[v] = std::vector<std::unique_ptr<cvrp_ls_operator>>(V);
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
    // Exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first or // This operator is symmetric.
          _sol[s_t.first].size() == 0 or _sol[s_t.second].size() == 0) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size(); ++t_rank) {
          cvrp_exchange
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<cvrp_exchange>(r);
          }
        }
      }
    }

    // CROSS-exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first or // This operator is symmetric.
          _sol[s_t.first].size() < 2 or _sol[s_t.second].size() < 2) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size() - 1;
             ++t_rank) {
          cvrp_cross_exchange
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<cvrp_cross_exchange>(r);
          }
        }
      }
    }

    // 2-opt* stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first) {
        // This operator is symmetric.
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        auto s_free_amount = _input._vehicles[s_t.first].capacity;
        s_free_amount -= _sol_state.fwd_amounts[s_t.first][s_rank];
        for (int t_rank = _sol[s_t.second].size() - 1; t_rank >= 0; --t_rank) {
          if (!(_sol_state.bwd_amounts[s_t.second][t_rank] <= s_free_amount)) {
            break;
          }
          cvrp_two_opt
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<cvrp_two_opt>(r);
          }
        }
      }
    }

    // Reverse 2-opt* stuff
    for (const auto& s_t : s_t_pairs) {
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        auto s_free_amount = _input._vehicles[s_t.first].capacity;
        s_free_amount -= _sol_state.fwd_amounts[s_t.first][s_rank];
        for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size(); ++t_rank) {
          if (!(_sol_state.fwd_amounts[s_t.second][t_rank] <= s_free_amount)) {
            break;
          }
          cvrp_reverse_two_opt
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<cvrp_reverse_two_opt>(r);
          }
        }
      }
    }

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
          cvrp_relocate
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<cvrp_relocate>(r);
          }
        }
      }
    }

    // Or-opt stuff
    for (const auto& s_t : s_t_pairs) {
      if (_sol[s_t.first].size() < 2 or
          !(_sol_state.total_amount(s_t.second) + _double_amount_lower_bound <=
            _input._vehicles[s_t.second].capacity)) {
        // Don't try to put things in a full vehicle or from a
        // (near-)empty vehicle.
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        if (_sol_state.edge_gains[s_t.first][s_rank] <=
            best_gains[s_t.first][s_t.second]) {
          // Except if addition cost in route s_t.second is negative
          // (!!), overall gain can't exceed current known best gain.
          continue;
        }
        for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
          cvrp_or_opt
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<cvrp_or_opt>(r);
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

      run_tsp(best_source);
      run_tsp(best_target);

      // We need to run update_amounts before try_job_additions to
      // correctly evaluate amounts. No need to run it again after
      // since try_before_additions will subsequently fix amounts upon
      // each addition.
      _sol_state.update_amounts(_sol, best_source);
      _sol_state.update_amounts(_sol, best_target);

      try_job_additions(best_ops[best_source][best_target]
                          ->addition_candidates(),
                        0);

      // Running update_costs only after try_job_additions is fine.
      _sol_state.update_costs(_sol, best_source);
      _sol_state.update_costs(_sol, best_target);

      _sol_state.update_skills(_sol, best_source);
      _sol_state.update_skills(_sol, best_target);

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

void cvrp_local_search::run() {
  bool try_ls_step = true;
  bool first_step = true;

  unsigned current_nb_removal = 1;

  while (try_ls_step) {
    // A round of local search.
    run_ls_step();

    // Remember best known solution.
    auto current_unassigned = _sol_state.unassigned.size();
    cost_t current_cost = 0;
    for (std::size_t v = 0; v < _sol.size(); ++v) {
      current_cost += _sol_state.route_cost_for_vehicle(v, _sol[v]);
    }
    bool solution_improved =
      (current_unassigned < _best_unassigned or
       (current_unassigned == _best_unassigned and current_cost < _best_cost));

    if (solution_improved) {
      _best_unassigned = current_unassigned;
      _best_cost = current_cost;
      _best_sol = _sol;
    } else {
      if (!first_step) {
        ++current_nb_removal;
      }
    }

    // Try again on each improvement until we reach last job removal
    // level.
    try_ls_step = (current_nb_removal <= _max_nb_jobs_removal);

    if (try_ls_step) {
      // Get a looser situation by removing jobs.
      for (unsigned i = 0; i < current_nb_removal; ++i) {
        remove_from_routes();
        for (std::size_t v = 0; v < _sol.size(); ++v) {
          _sol_state.set_node_gains(_sol, v);
        }
      }

      // Refill jobs (requires updated amounts).
      for (std::size_t v = 0; v < _sol.size(); ++v) {
        _sol_state.update_amounts(_sol, v);
      }
      try_job_additions(_all_routes, 1.5);

      for (std::size_t v = 0; v < _sol.size(); ++v) {
        run_tsp(v);
      }

      // Reset what is needed in solution state.
      _sol_state.setup(_sol);
    }

    first_step = false;
  }

  _target_sol = _best_sol;
}

void cvrp_local_search::remove_from_routes() {
  // Store nearest job from and to any job in any route for constant
  // time access down the line.
  for (std::size_t v1 = 0; v1 < V; ++v1) {
    for (std::size_t v2 = 0; v2 < V; ++v2) {
      if (v2 == v1) {
        continue;
      }
      _sol_state.update_nearest_job_rank_in_routes(_sol, v1, v2);
    }
  }

  // Remove best node candidate from all routes.
  std::vector<std::pair<index_t, index_t>> routes_and_ranks;

  for (std::size_t v = 0; v < _sol.size(); ++v) {
    if (_sol[v].empty()) {
      continue;
    }

    // Try removing the best node (good gain on current route and
    // small cost to closest node in another compatible route).
    index_t best_rank = 0;
    gain_t best_gain = std::numeric_limits<gain_t>::min();

    for (std::size_t r = 0; r < _sol[v].size(); ++r) {
      gain_t best_relocate_distance = std::numeric_limits<gain_t>::max();

      auto current_index = _input._jobs[_sol[v][r]].index();

      for (std::size_t other_v = 0; other_v < _sol.size(); ++other_v) {
        if (other_v == v or !_input.vehicle_ok_with_job(other_v, _sol[v][r])) {
          continue;
        }
        gain_t relocate_distance = std::numeric_limits<gain_t>::max();

        if (_input._vehicles[other_v].has_start()) {
          auto start_index = _input._vehicles[other_v].start.get().index();
          gain_t start_cost = _m[start_index][current_index];
          relocate_distance = std::min(relocate_distance, start_cost);
        }
        if (_input._vehicles[other_v].has_end()) {
          auto end_index = _input._vehicles[other_v].end.get().index();
          gain_t end_cost = _m[current_index][end_index];
          relocate_distance = std::min(relocate_distance, end_cost);
        }
        if (_sol[other_v].size() != 0) {
          auto nearest_from_rank =
            _sol_state.nearest_job_rank_in_routes_from[v][other_v][r];
          auto nearest_from_index =
            _input._jobs[_sol[other_v][nearest_from_rank]].index();
          gain_t cost_from = _m[nearest_from_index][current_index];
          relocate_distance = std::min(relocate_distance, cost_from);

          auto nearest_to_rank =
            _sol_state.nearest_job_rank_in_routes_to[v][other_v][r];
          auto nearest_to_index =
            _input._jobs[_sol[other_v][nearest_to_rank]].index();
          gain_t cost_to = _m[current_index][nearest_to_index];
          relocate_distance = std::min(relocate_distance, cost_to);
        }

        if (relocate_distance < best_relocate_distance) {
          best_relocate_distance = relocate_distance;
        }
      }

      gain_t current_gain =
        _sol_state.node_gains[v][r] - best_relocate_distance;

      if (current_gain > best_gain) {
        best_gain = current_gain;
        best_rank = r;
      }
    }

    routes_and_ranks.push_back(std::make_pair(v, best_rank));
  }

  for (const auto& r_r : routes_and_ranks) {
    auto v = r_r.first;
    auto r = r_r.second;
    _sol_state.unassigned.insert(_sol[v][r]);
    _sol[v].erase(_sol[v].begin() + r);
  }
}

void cvrp_local_search::run_tsp(index_t route_rank) {
  if (_sol[route_rank].size() > 0) {
    auto before_cost = _sol_state.route_costs[route_rank];

    tsp p(_input, _sol[route_rank], route_rank);
    auto new_route = p.raw_solve(0, 1)[0];

    auto after_cost = _sol_state.route_cost_for_vehicle(route_rank, new_route);

    if (after_cost < before_cost) {
      _sol[route_rank] = std::move(new_route);
      _sol_state.route_costs[route_rank] = after_cost;
    }
  }
}

solution_indicators cvrp_local_search::indicators() const {
  solution_indicators si;

  si.unassigned = _best_unassigned;
  si.cost = _best_cost;
  si.used_vehicles = std::count_if(_best_sol.begin(),
                                   _best_sol.end(),
                                   [](const auto& r) { return !r.empty(); });
  return si;
}
