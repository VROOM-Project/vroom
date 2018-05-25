/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "2_opt.h"
#include "cross_exchange.h"
#include "exchange.h"
#include "local_search.h"
#include "or_opt.h"
#include "relocate.h"
#include "reverse_2_opt.h"

void cvrp_local_search::set_node_gains(index_t v) {
  ls_operator::node_gains[v] = std::vector<gain_t>(_sol[v].size());
  ls_operator::edge_costs_around_node[v] = std::vector<gain_t>(_sol[v].size());

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
  ls_operator::edge_costs_around_node[v][0] = edges_costs_around;

  gain_t current_gain = edges_costs_around - new_edge_cost;
  ls_operator::node_gains[v][0] = current_gain;
  gain_t best_gain = current_gain;
  ls_operator::node_candidates[v] = 0;

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
    ls_operator::edge_costs_around_node[v][i] = edges_costs_around;

    current_gain = edges_costs_around - _m[p_index][n_index];
    ls_operator::node_gains[v][i] = current_gain;

    if (current_gain > best_gain) {
      best_gain = current_gain;
      ls_operator::node_candidates[v] = i;
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
  ls_operator::edge_costs_around_node[v][last_rank] = edges_costs_around;

  current_gain = edges_costs_around - new_edge_cost;
  ls_operator::node_gains[v][last_rank] = current_gain;

  if (current_gain > best_gain) {
    ls_operator::node_candidates[v] = last_rank;
  }
}

void cvrp_local_search::set_edge_gains(index_t v) {
  std::size_t nb_edges = (_sol[v].size() < 2) ? 0 : _sol[v].size() - 1;

  ls_operator::edge_gains[v] = std::vector<gain_t>(nb_edges);
  ls_operator::edge_costs_around_edge[v] = std::vector<gain_t>(nb_edges);

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
  ls_operator::edge_costs_around_edge[v][0] = edges_costs_around;

  gain_t current_gain = edges_costs_around - new_edge_cost;
  ls_operator::edge_gains[v][0] = current_gain;
  gain_t best_gain = current_gain;
  ls_operator::edge_candidates[v] = 0;

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
    ls_operator::edge_costs_around_edge[v][i] = edges_costs_around;

    current_gain = edges_costs_around - _m[p_index][n_index];
    ls_operator::edge_gains[v][i] = current_gain;

    if (current_gain > best_gain) {
      best_gain = current_gain;
      ls_operator::edge_candidates[v] = i;
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
  ls_operator::edge_costs_around_edge[v][last_edge_rank] = edges_costs_around;

  current_gain = edges_costs_around - new_edge_cost;
  ls_operator::edge_gains[v][last_edge_rank] = current_gain;

  if (current_gain > best_gain) {
    ls_operator::edge_candidates[v] = last_edge_rank;
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
  ls_operator::fwd_costs[v] = std::vector<cost_t>(_sol[v].size());
  ls_operator::bwd_costs[v] = std::vector<cost_t>(_sol[v].size());

  auto& m = _input.get_matrix();
  cost_t current_fwd = 0;
  cost_t current_bwd = 0;

  index_t previous_index = 0; // dummy init
  if (!_sol[v].empty()) {
    previous_index = _input._jobs[_sol[v][0]].index();
    ls_operator::fwd_costs[v][0] = current_fwd;
    ls_operator::bwd_costs[v][0] = current_bwd;
  }

  for (std::size_t i = 1; i < _sol[v].size(); ++i) {
    auto current_index = _input._jobs[_sol[v][i]].index();
    current_fwd += m[previous_index][current_index];
    current_bwd += m[current_index][previous_index];
    ls_operator::fwd_costs[v][i] = current_fwd;
    ls_operator::bwd_costs[v][i] = current_bwd;
    previous_index = current_index;
  }
}

void cvrp_local_search::update_amounts(index_t v) {
  ls_operator::fwd_amounts[v] = std::vector<amount_t>(_sol[v].size());
  ls_operator::bwd_amounts[v] = std::vector<amount_t>(_sol[v].size());
  amount_t current_amount(_input.amount_size());

  for (std::size_t i = 0; i < _sol[v].size(); ++i) {
    current_amount += _input._jobs[_sol[v][i]].amount;
    ls_operator::fwd_amounts[v][i] = current_amount;
  }

  std::transform(ls_operator::fwd_amounts[v].cbegin(),
                 ls_operator::fwd_amounts[v].cend(),
                 ls_operator::bwd_amounts[v].begin(),
                 [&](const auto& a) {
                   auto total_amount = ls_operator::fwd_amounts[v].back();
                   return total_amount - a;
                 });
}

amount_t cvrp_local_search::total_amount(index_t v) {
  amount_t v_amount(_input.amount_size());
  if (!ls_operator::fwd_amounts[v].empty()) {
    v_amount = ls_operator::fwd_amounts[v].back();
  }
  return v_amount;
}

void cvrp_local_search::update_nearest_job_rank_in_routes(index_t v1,
                                                          index_t v2) {
  _nearest_job_rank_in_routes_from[v1][v2] =
    std::vector<index_t>(_sol[v1].size());
  _nearest_job_rank_in_routes_to[v1][v2] =
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

    _nearest_job_rank_in_routes_from[v1][v2][r1] = best_from_rank;
    _nearest_job_rank_in_routes_to[v1][v2][r1] = best_to_rank;
  }
}

cvrp_local_search::cvrp_local_search(const input& input, raw_solution& sol)
  : _input(input),
    _m(_input.get_matrix()),
    V(_input._vehicles.size()),
    _sol(sol),
    _amount_lower_bound(_input.get_amount_lower_bound()),
    _double_amount_lower_bound(_amount_lower_bound + _amount_lower_bound),
    _nearest_job_rank_in_routes_from(V, std::vector<std::vector<index_t>>(V)),
    _nearest_job_rank_in_routes_to(V, std::vector<std::vector<index_t>>(V)),
    _log(false),
    _ls_step(0) {

  // Initialize amounts and costs storage.
  ls_operator::fwd_amounts = std::vector<std::vector<amount_t>>(_sol.size());
  ls_operator::bwd_amounts = std::vector<std::vector<amount_t>>(_sol.size());
  ls_operator::fwd_costs = std::vector<std::vector<cost_t>>(_sol.size());
  ls_operator::bwd_costs = std::vector<std::vector<cost_t>>(_sol.size());
  for (std::size_t v = 0; v < _sol.size(); ++v) {
    update_amounts(v);
    update_costs(v);
  }

  // Initialize unassigned jobs.
  std::generate_n(std::inserter(_unassigned, _unassigned.end()),
                  _input._jobs.size(),
                  [] {
                    static int x = 0;
                    return x++;
                  });

  for (const auto& s : _sol) {
    for (const auto i : s) {
      _unassigned.erase(i);
    }
  }

  std::cout << "Unassigned jobs: ";
  for (auto i : _unassigned) {
    std::cout << _input._jobs[i].id << " (" << _input._jobs[i].amount[0]
              << ") ; ";
  }
  std::cout << std::endl;

  // Logging stuff, todo remove.
  std::cout << "Amount lower bound: ";
  for (std::size_t r = 0; r < _amount_lower_bound.size(); ++r) {
    std::cout << _amount_lower_bound[r];
  }
  std::cout << std::endl;

  for (std::size_t v = 0; v < sol.size(); ++v) {
    if (ls_operator::fwd_amounts[v].empty()) {
      assert(_sol[v].empty());
      continue;
    }
    auto& capacity = _input._vehicles[v].capacity;
    std::cout << "Amount for vehicle " << _input._vehicles[v].id << " (at rank "
              << v << "): ";
    auto& v_amount = ls_operator::fwd_amounts[v].back();
    for (std::size_t r = 0; r < v_amount.size(); ++r) {
      std::cout << v_amount[r] << " / " << capacity[r] << " ; ";
    }
    std::cout << std::endl;
  }

  // Initialize storage and find best candidate for job/edge pop in
  // each route.
  ls_operator::edge_costs_around_node = std::vector<std::vector<gain_t>>(V);
  ls_operator::node_gains = std::vector<std::vector<gain_t>>(V);
  ls_operator::node_candidates = std::vector<index_t>(V);
  ls_operator::edge_costs_around_edge = std::vector<std::vector<gain_t>>(V);
  ls_operator::edge_gains = std::vector<std::vector<gain_t>>(V);
  ls_operator::edge_candidates = std::vector<index_t>(V);

  for (std::size_t v = 0; v < V; ++v) {
    set_node_gains(v);
    set_edge_gains(v);
  }

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
}

void cvrp_local_search::try_job_additions(const std::vector<index_t>& routes) {
  auto& m = _input.get_matrix();
  bool job_added;

  do {
    auto best_cost = std::numeric_limits<gain_t>::max();
    index_t best_job;
    index_t best_route;
    index_t best_rank;

    for (const auto v : routes) {
      const auto& v_target = _input._vehicles[v];
      const amount_t v_amount = total_amount(v);

      for (const auto j : _unassigned) {
        auto& current_amount = _input._jobs[j].amount;

        if (_input.vehicle_ok_with_job(v, j) and
            v_amount + current_amount <= _input._vehicles[v].capacity) {
          auto index_j = _input._jobs[j].index();

          for (std::size_t r = 0; r < _sol[v].size(); ++r) {
            // Check cost of adding unassigned job at rank r in route
            // v. Same logic as in relocate::compute_gain.
            gain_t previous_cost = 0;
            gain_t next_cost = 0;
            gain_t old_edge_cost = 0;

            if (r == _sol[v].size()) {
              if (_sol[v].size() == 0) {
                // Adding job to an empty route.
                if (v_target.has_start()) {
                  previous_cost = m[v_target.start.get().index()][index_j];
                }
                if (v_target.has_end()) {
                  next_cost = m[index_j][v_target.end.get().index()];
                }
              } else {
                // Adding job past the end after a real job.
                auto p_index = _input._jobs[_sol[v][r - 1]].index();
                previous_cost = m[p_index][index_j];
                if (v_target.has_end()) {
                  auto n_index = v_target.end.get().index();
                  old_edge_cost = m[p_index][n_index];
                  next_cost = m[index_j][n_index];
                }
              }
            } else {
              // Adding before one of the jobs.
              auto n_index = _input._jobs[_sol[v][r]].index();
              next_cost = m[index_j][n_index];

              if (r == 0) {
                if (v_target.has_start()) {
                  auto p_index = v_target.start.get().index();
                  previous_cost = m[p_index][index_j];
                  old_edge_cost = m[p_index][n_index];
                }
              } else {
                auto p_index = _input._jobs[_sol[v][r - 1]].index();
                previous_cost = m[p_index][index_j];
                old_edge_cost = m[p_index][n_index];
              }
            }

            gain_t current_cost = old_edge_cost - previous_cost - next_cost;
            if (current_cost < best_cost) {
              best_cost = current_cost;
              best_job = j;
              best_route = v;
              best_rank = r;
            }
          }
        }
      }
    }

    job_added = (best_cost < std::numeric_limits<gain_t>::max());

    if (job_added) {
      std::cout << "- Adding job: " << _input._jobs[best_job].id << " at rank "
                << best_rank << " in route for vehicle "
                << _input._vehicles[best_route].id << "." << std::endl;
      _sol[best_route].insert(_sol[best_route].begin() + best_rank, best_job);

      // Update amounts after addition.
      const auto& job_amount = _input._jobs[best_job].amount;
      auto& best_fwd_amounts = ls_operator::fwd_amounts[best_route];
      auto previous_cumul = (best_rank == 0) ? amount_t(_input.amount_size())
                                             : best_fwd_amounts[best_rank - 1];
      best_fwd_amounts.insert(best_fwd_amounts.begin() + best_rank,
                              previous_cumul + job_amount);
      std::for_each(best_fwd_amounts.begin() + best_rank + 1,
                    best_fwd_amounts.end(),
                    [&](auto& a) { a += job_amount; });

      auto& best_bwd_amounts = ls_operator::bwd_amounts[best_route];
      best_bwd_amounts.insert(best_bwd_amounts.begin() + best_rank,
                              amount_t(_input.amount_size())); // dummy init
      for (std::size_t i = 0; i <= best_rank; ++i) {
        auto total_amount = ls_operator::fwd_amounts[best_route].back();
        ls_operator::bwd_amounts[best_route][i] =
          total_amount - ls_operator::fwd_amounts[best_route][i];
      }

      _unassigned.erase(best_job);
    }
  } while (job_added);
}

void cvrp_local_search::run(unsigned nb_threads) {
  std::cout << "* Running CVRP local search." << std::endl;

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
          relocate r(_input, _sol, s_t.first, s_rank, s_t.second, t_rank);
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
          exchange r(_input, _sol, s_t.first, s_rank, s_t.second, t_rank);
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
          or_opt r(_input, _sol, s_t.first, s_rank, s_t.second, t_rank);
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
          cross_exchange r(_input, _sol, s_t.first, s_rank, s_t.second, t_rank);
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
        s_free_amount -= ls_operator::fwd_amounts[s_t.first][s_rank];
        for (int t_rank = _sol[s_t.second].size() - 1; t_rank >= 0; --t_rank) {
          if (!(ls_operator::bwd_amounts[s_t.second][t_rank] <=
                s_free_amount)) {
            break;
          }
          two_opt r(_input, _sol, s_t.first, s_rank, s_t.second, t_rank);
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
        s_free_amount -= ls_operator::fwd_amounts[s_t.first][s_rank];
        for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size(); ++t_rank) {
          if (!(ls_operator::fwd_amounts[s_t.second][t_rank] <=
                s_free_amount)) {
            break;
          }
          reverse_two_opt r(_input,
                            _sol,
                            s_t.first,
                            s_rank,
                            s_t.second,
                            t_rank);
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

      best_ops[best_source][best_target]->log();

      best_ops[best_source][best_target]->apply();

      log_solution();

      run_tsp(best_source, 1);
      run_tsp(best_target, 1);

      // We need to run update_amounts before try_job_additions to
      // correctly evaluate amounts. No need to run it again after
      // since try_before_additions will subsequently fix amounts upon
      // each addition.
      update_amounts(best_source);
      update_amounts(best_target);

      try_job_additions(
        best_ops[best_source][best_target]->addition_candidates());

      // Running update_costs only after try_job_additions is fine.
      update_costs(best_source);
      update_costs(best_target);

      log_solution();

      // Update candidates.
      set_node_gains(best_source);
      set_node_gains(best_target);
      set_edge_gains(best_source);
      set_edge_gains(best_target);

      // Set gains to zero for what needs to be recomputed in the next round.
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

void cvrp_local_search::run_tsp(index_t route_rank, unsigned nb_threads) {
  if (_sol[route_rank].size() > 0) {
    auto before_cost = route_cost_for_vehicle(route_rank, _sol[route_rank]);

    tsp p(_input, _sol[route_rank], nb_threads);
    auto new_route = p.solve(nb_threads)[0];

    auto after_cost = route_cost_for_vehicle(route_rank, new_route);

    if (after_cost < before_cost) {
      _sol[route_rank] = std::move(new_route);
      std::cout << "Rearrange gain: " << before_cost - after_cost << std::endl;
    }
  }
}
