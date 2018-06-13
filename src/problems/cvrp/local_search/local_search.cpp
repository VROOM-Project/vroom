/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include <boost/log/trivial.hpp>

#include "problems/cvrp/local_search/2_opt.h"
#include "problems/cvrp/local_search/cross_exchange.h"
#include "problems/cvrp/local_search/exchange.h"
#include "problems/cvrp/local_search/local_search.h"
#include "problems/cvrp/local_search/or_opt.h"
#include "problems/cvrp/local_search/relocate.h"
#include "problems/cvrp/local_search/reverse_2_opt.h"
#include "problems/tsp/tsp.h"

cvrp_local_search::cvrp_local_search(const input& input, raw_solution& sol)
  : _input(input),
    _m(_input.get_matrix()),
    V(_input._vehicles.size()),
    _amount_lower_bound(_input.get_amount_lower_bound()),
    _double_amount_lower_bound(_amount_lower_bound + _amount_lower_bound),
    _all_routes(V),
    _target_sol(sol),
    _sol(sol),
    _sol_state(_sol.size()),
    _best_sol(sol),
    _log(false),
    _ls_step(0) {
  // Initialize all route indices.
  std::iota(_all_routes.begin(), _all_routes.end(), 0);

  // Initialize solution state.
  setup();

  // Initialize unassigned jobs.
  index_t x = 0;
  std::generate_n(std::inserter(_unassigned, _unassigned.end()),
                  _input._jobs.size(),
                  [&] { return x++; });

  for (const auto& s : _sol) {
    for (const auto i : s) {
      _unassigned.erase(i);
    }
  }

  _best_unassigned = _unassigned.size();
  _best_cost = 0;
  for (std::size_t v = 0; v < _sol.size(); ++v) {
    _best_cost += route_cost_for_vehicle(v, _sol[v]);
  }
}

void cvrp_local_search::setup() {
  for (std::size_t v = 0; v < _sol.size(); ++v) {
    update_amounts(v);
    update_costs(v);
    update_skills(v);
    set_node_gains(v);
    set_edge_gains(v);

    _sol_state.route_costs[v] = route_cost_for_vehicle(v, _sol[v]);
  }
}

void cvrp_local_search::set_node_gains(index_t v) {
  _sol_state.node_gains[v] = std::vector<gain_t>(_sol[v].size());
  _sol_state.edge_costs_around_node[v] = std::vector<gain_t>(_sol[v].size());

  if (_sol[v].size() == 0) {
    return;
  }

  // Handling first job is special due to potential open tours.
  index_t p_index;
  index_t c_index = _input._jobs[_sol[v][0]].index();
  index_t n_index;

  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t new_edge_cost = 0;

  if (_input._vehicles[v].has_start()) {
    // There is a previous step before job at rank 0.
    p_index = _input._vehicles[v].start.get().index();
    previous_cost = _m[p_index][c_index];

    // Update next_cost with next job or end.
    if (_sol[v].size() > 1) {
      n_index = _input._jobs[_sol[v][1]].index();
      next_cost = _m[c_index][n_index];
      new_edge_cost = _m[p_index][n_index];
    } else {
      // _sol[v].size() is 1 and first job is also the last.
      if (_input._vehicles[v].has_end()) {
        next_cost = _m[c_index][_input._vehicles[v].end.get().index()];
      }
    }
  } else {
    // There is a next cost either to next job or to end of route, but
    // no new edge.
    if (_sol[v].size() > 1) {
      n_index = _input._jobs[_sol[v][1]].index();
    } else {
      assert(_input._vehicles[v].has_end());
      n_index = _input._vehicles[v].end.get().index();
    }
    next_cost = _m[c_index][n_index];
  }

  gain_t edges_costs_around = previous_cost + next_cost;
  _sol_state.edge_costs_around_node[v][0] = edges_costs_around;

  gain_t current_gain = edges_costs_around - new_edge_cost;
  _sol_state.node_gains[v][0] = current_gain;
  gain_t best_gain = current_gain;
  _sol_state.node_candidates[v] = 0;

  if (_sol[v].size() == 1) {
    // No more jobs.
    return;
  }

  // Handle jobs that always have a previous and next job.
  for (std::size_t i = 1; i < _sol[v].size() - 1; ++i) {
    // Compute potential gain to relocate current job.
    p_index = _input._jobs[_sol[v][i - 1]].index();
    c_index = _input._jobs[_sol[v][i]].index();
    n_index = _input._jobs[_sol[v][i + 1]].index();

    edges_costs_around = _m[p_index][c_index] + _m[c_index][n_index];
    _sol_state.edge_costs_around_node[v][i] = edges_costs_around;

    current_gain = edges_costs_around - _m[p_index][n_index];
    _sol_state.node_gains[v][i] = current_gain;

    if (current_gain > best_gain) {
      best_gain = current_gain;
      _sol_state.node_candidates[v] = i;
    }
  }

  // Handling last job is special due to potential open tours.
  auto last_rank = _sol[v].size() - 1;
  c_index = _input._jobs[_sol[v][last_rank]].index();

  previous_cost = 0;
  next_cost = 0;
  new_edge_cost = 0;

  if (_input._vehicles[v].has_end()) {
    // There is a next step after last job.
    n_index = _input._vehicles[v].end.get().index();
    next_cost = _m[c_index][n_index];

    if (_sol[v].size() > 1) {
      p_index = _input._jobs[_sol[v][last_rank - 1]].index();
      previous_cost = _m[p_index][c_index];
      new_edge_cost = _m[p_index][n_index];
    }
  } else {
    // There is a previous cost either from previous job or from start
    // of route, but no new edge.
    if (_sol[v].size() > 1) {
      p_index = _input._jobs[_sol[v][last_rank - 1]].index();
    } else {
      assert(_input._vehicles[v].has_start());
      p_index = _input._vehicles[v].start.get().index();
    }
    previous_cost = _m[p_index][c_index];
  }

  edges_costs_around = previous_cost + next_cost;
  _sol_state.edge_costs_around_node[v][last_rank] = edges_costs_around;

  current_gain = edges_costs_around - new_edge_cost;
  _sol_state.node_gains[v][last_rank] = current_gain;

  if (current_gain > best_gain) {
    _sol_state.node_candidates[v] = last_rank;
  }
}

void cvrp_local_search::set_edge_gains(index_t v) {
  std::size_t nb_edges = (_sol[v].size() < 2) ? 0 : _sol[v].size() - 1;

  _sol_state.edge_gains[v] = std::vector<gain_t>(nb_edges);
  _sol_state.edge_costs_around_edge[v] = std::vector<gain_t>(nb_edges);

  if (_sol[v].size() < 2) {
    return;
  }

  // Handling first edge is special due to potential open tours.
  index_t p_index;
  index_t c_index = _input._jobs[_sol[v][0]].index();
  index_t after_c_index = _input._jobs[_sol[v][1]].index();
  index_t n_index;

  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t new_edge_cost = 0;

  if (_input._vehicles[v].has_start()) {
    // There is a previous step before job at rank 0.
    p_index = _input._vehicles[v].start.get().index();
    previous_cost = _m[p_index][c_index];

    // Update next_cost with next job or end.
    if (_sol[v].size() > 2) {
      n_index = _input._jobs[_sol[v][2]].index();
      next_cost = _m[after_c_index][n_index];
      new_edge_cost = _m[p_index][n_index];
    } else {
      // _sol[v].size() is 2 and first edge is also the last.
      if (_input._vehicles[v].has_end()) {
        next_cost = _m[after_c_index][_input._vehicles[v].end.get().index()];
      }
    }
  } else {
    // There is a next cost either to next job or to end of route, but
    // no new edge.
    if (_sol[v].size() > 2) {
      n_index = _input._jobs[_sol[v][2]].index();
    } else {
      assert(_input._vehicles[v].has_end());
      n_index = _input._vehicles[v].end.get().index();
    }
    next_cost = _m[after_c_index][n_index];
  }

  gain_t edges_costs_around = previous_cost + next_cost;
  _sol_state.edge_costs_around_edge[v][0] = edges_costs_around;

  gain_t current_gain = edges_costs_around - new_edge_cost;
  _sol_state.edge_gains[v][0] = current_gain;
  gain_t best_gain = current_gain;
  _sol_state.edge_candidates[v] = 0;

  if (_sol[v].size() == 2) {
    // No more edges.
    return;
  }

  // Handle edges that always have a previous and next job.
  for (std::size_t i = 1; i < nb_edges - 1; ++i) {
    // Compute potential gain to relocate edge from current to next
    // job.
    p_index = _input._jobs[_sol[v][i - 1]].index();
    c_index = _input._jobs[_sol[v][i]].index();
    after_c_index = _input._jobs[_sol[v][i + 1]].index();
    n_index = _input._jobs[_sol[v][i + 2]].index();

    edges_costs_around = _m[p_index][c_index] + _m[after_c_index][n_index];
    _sol_state.edge_costs_around_edge[v][i] = edges_costs_around;

    current_gain = edges_costs_around - _m[p_index][n_index];
    _sol_state.edge_gains[v][i] = current_gain;

    if (current_gain > best_gain) {
      best_gain = current_gain;
      _sol_state.edge_candidates[v] = i;
    }
  }

  // Handling last edge is special due to potential open tours.
  auto last_edge_rank = nb_edges - 1;
  c_index = _input._jobs[_sol[v][last_edge_rank]].index();
  after_c_index = _input._jobs[_sol[v][last_edge_rank + 1]].index();

  previous_cost = 0;
  next_cost = 0;
  new_edge_cost = 0;

  if (_input._vehicles[v].has_end()) {
    // There is a next step after last job.
    n_index = _input._vehicles[v].end.get().index();
    next_cost = _m[after_c_index][n_index];

    if (_sol[v].size() > 2) {
      p_index = _input._jobs[_sol[v][last_edge_rank - 1]].index();
      previous_cost = _m[p_index][c_index];
      new_edge_cost = _m[p_index][n_index];
    }
  } else {
    // There is a previous cost either from previous job or from start
    // of route, but no new edge.
    if (_sol[v].size() > 2) {
      p_index = _input._jobs[_sol[v][last_edge_rank - 1]].index();
    } else {
      assert(_input._vehicles[v].has_start());
      p_index = _input._vehicles[v].start.get().index();
    }
    previous_cost = _m[p_index][c_index];
  }

  edges_costs_around = previous_cost + next_cost;
  _sol_state.edge_costs_around_edge[v][last_edge_rank] = edges_costs_around;

  current_gain = edges_costs_around - new_edge_cost;
  _sol_state.edge_gains[v][last_edge_rank] = current_gain;

  if (current_gain > best_gain) {
    _sol_state.edge_candidates[v] = last_edge_rank;
  }
}

void cvrp_local_search::log_solution() {
  if (_log) {
    write_to_json(_input.format_solution(_sol),
                  false,
                  "ls_log_" + std::to_string(_ls_step++) + "_sol.json");
  }
}

void cvrp_local_search::update_costs(index_t v) {
  _sol_state.fwd_costs[v] = std::vector<cost_t>(_sol[v].size());
  _sol_state.bwd_costs[v] = std::vector<cost_t>(_sol[v].size());

  cost_t current_fwd = 0;
  cost_t current_bwd = 0;

  index_t previous_index = 0; // dummy init
  if (!_sol[v].empty()) {
    previous_index = _input._jobs[_sol[v][0]].index();
    _sol_state.fwd_costs[v][0] = current_fwd;
    _sol_state.bwd_costs[v][0] = current_bwd;
  }

  for (std::size_t i = 1; i < _sol[v].size(); ++i) {
    auto current_index = _input._jobs[_sol[v][i]].index();
    current_fwd += _m[previous_index][current_index];
    current_bwd += _m[current_index][previous_index];
    _sol_state.fwd_costs[v][i] = current_fwd;
    _sol_state.bwd_costs[v][i] = current_bwd;
    previous_index = current_index;
  }
}

void cvrp_local_search::update_amounts(index_t v) {
  _sol_state.fwd_amounts[v] = std::vector<amount_t>(_sol[v].size());
  _sol_state.bwd_amounts[v] = std::vector<amount_t>(_sol[v].size());
  amount_t current_amount(_input.amount_size());

  for (std::size_t i = 0; i < _sol[v].size(); ++i) {
    current_amount += _input._jobs[_sol[v][i]].amount;
    _sol_state.fwd_amounts[v][i] = current_amount;
  }

  std::transform(_sol_state.fwd_amounts[v].cbegin(),
                 _sol_state.fwd_amounts[v].cend(),
                 _sol_state.bwd_amounts[v].begin(),
                 [&](const auto& a) {
                   auto total_amount = _sol_state.fwd_amounts[v].back();
                   return total_amount - a;
                 });
}

void cvrp_local_search::update_skills(index_t v1) {
  for (std::size_t v2 = 0; v2 < V; ++v2) {
    if (v1 == v2) {
      continue;
    }

    auto fwd =
      std::find_if_not(_sol[v1].begin(), _sol[v1].end(), [&](auto j_rank) {
        return _input.vehicle_ok_with_job(v2, j_rank);
      });
    _sol_state.fwd_skill_rank[v1][v2] = std::distance(_sol[v1].begin(), fwd);

    auto bwd =
      std::find_if_not(_sol[v1].rbegin(), _sol[v1].rend(), [&](auto j_rank) {
        return _input.vehicle_ok_with_job(v2, j_rank);
      });
    _sol_state.bwd_skill_rank[v1][v2] =
      _sol[v1].size() - std::distance(_sol[v1].rbegin(), bwd);
  }
}

amount_t cvrp_local_search::total_amount(index_t v) {
  amount_t v_amount(_input.amount_size());
  if (!_sol_state.fwd_amounts[v].empty()) {
    v_amount = _sol_state.fwd_amounts[v].back();
  }
  return v_amount;
}

void cvrp_local_search::update_nearest_job_rank_in_routes(index_t v1,
                                                          index_t v2) {
  _sol_state.nearest_job_rank_in_routes_from[v1][v2] =
    std::vector<index_t>(_sol[v1].size());
  _sol_state.nearest_job_rank_in_routes_to[v1][v2] =
    std::vector<index_t>(_sol[v1].size());

  for (std::size_t r1 = 0; r1 < _sol[v1].size(); ++r1) {
    index_t index_r1 = _input._jobs[_sol[v1][r1]].index();

    auto min_from = std::numeric_limits<cost_t>::max();
    auto min_to = std::numeric_limits<cost_t>::max();
    index_t best_from_rank = 0;
    index_t best_to_rank = 0;

    for (std::size_t r2 = 0; r2 < _sol[v2].size(); ++r2) {
      index_t index_r2 = _input._jobs[_sol[v2][r2]].index();
      if (_m[index_r1][index_r2] < min_from) {
        min_from = _m[index_r1][index_r2];
        best_from_rank = r2;
      }
      if (_m[index_r2][index_r1] < min_to) {
        min_to = _m[index_r2][index_r1];
        best_to_rank = r2;
      }
    }

    _sol_state.nearest_job_rank_in_routes_from[v1][v2][r1] = best_from_rank;
    _sol_state.nearest_job_rank_in_routes_to[v1][v2][r1] = best_to_rank;
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

    for (const auto j : _unassigned) {
      auto& current_amount = _input._jobs[j].amount;
      std::vector<gain_t> best_costs(routes.size(),
                                     std::numeric_limits<gain_t>::max());
      std::vector<index_t> best_ranks(routes.size());

      for (std::size_t i = 0; i < routes.size(); ++i) {
        auto v = routes[i];
        const auto& v_target = _input._vehicles[v];
        const amount_t v_amount = total_amount(v);

        if (_input.vehicle_ok_with_job(v, j) and
            v_amount + current_amount <= _input._vehicles[v].capacity) {
          auto index_j = _input._jobs[j].index();

          for (std::size_t r = 0; r <= _sol[v].size(); ++r) {
            // Check cost of adding unassigned job at rank r in route
            // v. Same logic as in relocate::compute_gain.
            gain_t previous_cost = 0;
            gain_t next_cost = 0;
            gain_t old_edge_cost = 0;

            if (r == _sol[v].size()) {
              if (_sol[v].size() == 0) {
                // Adding job to an empty route.
                if (v_target.has_start()) {
                  previous_cost = _m[v_target.start.get().index()][index_j];
                }
                if (v_target.has_end()) {
                  next_cost = _m[index_j][v_target.end.get().index()];
                }
              } else {
                // Adding job past the end after a real job.
                auto p_index = _input._jobs[_sol[v][r - 1]].index();
                previous_cost = _m[p_index][index_j];
                if (v_target.has_end()) {
                  auto n_index = v_target.end.get().index();
                  old_edge_cost = _m[p_index][n_index];
                  next_cost = _m[index_j][n_index];
                }
              }
            } else {
              // Adding before one of the jobs.
              auto n_index = _input._jobs[_sol[v][r]].index();
              next_cost = _m[index_j][n_index];

              if (r == 0) {
                if (v_target.has_start()) {
                  auto p_index = v_target.start.get().index();
                  previous_cost = _m[p_index][index_j];
                  old_edge_cost = _m[p_index][n_index];
                }
              } else {
                auto p_index = _input._jobs[_sol[v][r - 1]].index();
                previous_cost = _m[p_index][index_j];
                old_edge_cost = _m[p_index][n_index];
              }
            }

            gain_t current_cost = previous_cost + next_cost - old_edge_cost;

            if (current_cost < best_costs[i]) {
              best_costs[i] = current_cost;
              best_ranks[i] = r;
            }
          }
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
        for (std::size_t i2 = 0; i2 < routes.size(); ++i2) {
          if (i == i2) {
            continue;
          }
          regret_cost = std::min(regret_cost, best_costs[i2]);
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
      BOOST_LOG_TRIVIAL(trace) << "- Adding job: " << _input._jobs[best_job].id
                               << " at rank " << best_rank
                               << " in route for vehicle "
                               << _input._vehicles[best_route].id << ".";
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
      _sol_state.route_costs[best_route] =
        route_cost_for_vehicle(best_route, _sol[best_route]);

      _unassigned.erase(best_job);

      log_solution();
    }
  } while (job_added);
}

void cvrp_local_search::run_ls_step() {
  BOOST_LOG_TRIVIAL(trace) << "* Running CVRP local search step.";

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

  log_solution();

  gain_t best_gain = 1;

  while (best_gain > 0) {
    // Relocate stuff
    for (const auto& s_t : s_t_pairs) {
      if (_input._vehicles[s_t.second].capacity <
          total_amount(s_t.second) + _amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() == 0) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
          relocate
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<relocate>(r);
          }
        }
      }
    }

    // Exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first) {
        // This operator is symmetric.
        continue;
      }
      if ((_sol[s_t.first].size() == 0) or (_sol[s_t.second].size() == 0)) {
        continue;
      }

      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size(); ++t_rank) {
          exchange
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<exchange>(r);
          }
        }
      }
    }

    // Or-opt stuff
    for (const auto& s_t : s_t_pairs) {
      if (_input._vehicles[s_t.second].capacity <
          total_amount(s_t.second) + _double_amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() < 2) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
          or_opt
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<or_opt>(r);
          }
        }
      }
    }

    // CROSS-exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if (s_t.second <= s_t.first) {
        // This operator is symmetric.
        continue;
      }

      if ((_sol[s_t.first].size() < 2) or (_sol[s_t.second].size() < 2)) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size() - 1;
             ++t_rank) {
          cross_exchange
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<cross_exchange>(r);
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
          two_opt
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] = std::make_unique<two_opt>(r);
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
          reverse_two_opt
            r(_input, _sol, _sol_state, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<reverse_two_opt>(r);
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

      // best_ops[best_source][best_target]->log();

      best_ops[best_source][best_target]->apply();

      log_solution();

      // Update route costs.
      auto previous_cost = _sol_state.route_costs[best_source] +
                           _sol_state.route_costs[best_target];
      _sol_state.route_costs[best_source] =
        route_cost_for_vehicle(best_source, _sol[best_source]);
      _sol_state.route_costs[best_target] =
        route_cost_for_vehicle(best_target, _sol[best_target]);
      auto new_cost = _sol_state.route_costs[best_source] +
                      _sol_state.route_costs[best_target];
      assert(new_cost + best_gain == previous_cost);

      run_tsp(best_source);
      run_tsp(best_target);

      // We need to run update_amounts before try_job_additions to
      // correctly evaluate amounts. No need to run it again after
      // since try_before_additions will subsequently fix amounts upon
      // each addition.
      update_amounts(best_source);
      update_amounts(best_target);

      try_job_additions(best_ops[best_source][best_target]
                          ->addition_candidates(),
                        0);

      log_solution();

      // Running update_costs only after try_job_additions is fine.
      update_costs(best_source);
      update_costs(best_target);

      update_skills(best_source);
      update_skills(best_target);

      // Update candidates.
      set_node_gains(best_source);
      set_node_gains(best_target);
      set_edge_gains(best_source);
      set_edge_gains(best_target);

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
  bool first_try = true;

  while (try_ls_step) {
    // A round of local search.
    run_ls_step();

    // Remember best known solution.
    auto current_unassigned = _unassigned.size();
    cost_t current_cost = 0;
    for (std::size_t v = 0; v < _sol.size(); ++v) {
      current_cost += route_cost_for_vehicle(v, _sol[v]);
    }
    bool solution_improved =
      (current_unassigned < _best_unassigned or
       (current_unassigned == _best_unassigned and current_cost < _best_cost));
    if (solution_improved) {
      _best_unassigned = current_unassigned;
      _best_cost = current_cost;
      _best_sol = _sol;
    }

    // Try again on each improvement, but also in case first local
    // search step did not change anything.
    try_ls_step = solution_improved or first_try;

    if (try_ls_step) {
      // Get a looser situation by removing jobs.
      remove_from_routes();

      // Refill jobs (requires updated amounts).
      for (std::size_t v = 0; v < _sol.size(); ++v) {
        update_amounts(v);
      }
      try_job_additions(_all_routes, 1.5);

      for (std::size_t v = 0; v < _sol.size(); ++v) {
        run_tsp(v);
      }

      // Reset what is needed in solution state.
      setup();

      log_solution();
    }

    first_try = false;
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
      update_nearest_job_rank_in_routes(v1, v2);
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
    _unassigned.insert(_sol[v][r]);
    _sol[v].erase(_sol[v].begin() + r);

    log_solution();
  }
}

cost_t
cvrp_local_search::route_cost_for_vehicle(index_t vehicle_rank,
                                          const std::vector<index_t>& route) {
  const auto& v = _input._vehicles[vehicle_rank];
  auto cost = 0;

  if (route.size() > 0) {
    if (v.has_start()) {
      cost += _m[v.start.get().index()][_input._jobs[route.front()].index()];
    }

    index_t previous = route.front();
    for (auto it = ++route.cbegin(); it != route.cend(); ++it) {
      cost += _m[_input._jobs[previous].index()][_input._jobs[*it].index()];
      previous = *it;
    }

    if (v.has_end()) {
      cost += _m[_input._jobs[route.back()].index()][v.end.get().index()];
    }
  }

  return cost;
}

void cvrp_local_search::run_tsp(index_t route_rank) {
  if (_sol[route_rank].size() > 0) {
    auto before_cost = _sol_state.route_costs[route_rank];

    tsp p(_input, _sol[route_rank], route_rank);
    auto new_route = p.solve(1)[0];

    auto after_cost = route_cost_for_vehicle(route_rank, new_route);

    if (after_cost < before_cost) {
      _sol[route_rank] = std::move(new_route);
      _sol_state.route_costs[route_rank] = after_cost;
      BOOST_LOG_TRIVIAL(trace) << "Rearrange gain: " << before_cost - after_cost
                               << std::endl;
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
