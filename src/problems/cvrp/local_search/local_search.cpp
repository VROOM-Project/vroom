/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>
#include <unordered_set>

#include "cross_exchange.h"
#include "exchange.h"
#include "local_search.h"
#include "or_opt.h"
#include "relocate.h"

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
    _amounts(sol.size(), amount_t(input.amount_size())),
    _amount_lower_bound(_input.get_amount_lower_bound()),
    _double_amount_lower_bound(_amount_lower_bound + _amount_lower_bound),
    _nearest_job_rank_in_routes_from(V, std::vector<std::vector<index_t>>(V)),
    _nearest_job_rank_in_routes_to(V, std::vector<std::vector<index_t>>(V)),
    _log(false),
    _ls_step(0) {

  std::cout << "Amount lower bound: ";
  for (std::size_t r = 0; r < _amount_lower_bound.size(); ++r) {
    std::cout << _amount_lower_bound[r];
  }
  std::cout << std::endl;

  for (std::size_t i = 0; i < sol.size(); ++i) {
    for (const auto rank : sol[i]) {
      _amounts[i] += _input._jobs[rank].amount;
    }
    auto& capacity = _input._vehicles[i].capacity;

    std::cout << "Amount for vehicle " << _input._vehicles[i].id << " (at rank "
              << i << "): ";
    for (std::size_t r = 0; r < _amounts[i].size(); ++r) {
      std::cout << _amounts[i][r] << " / " << capacity[r] << " ; ";
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
      update_nearest_job_rank_in_routes(v1, v2);
    }
  }
}

void cvrp_local_search::run_with_fixed_source_and_target() {
  std::cout << "Running CVRP local search with fixed source and target."
            << std::endl;

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

  if (_log) {
    write_to_json(_input.format_solution(_sol),
                  false,
                  "ls_log_" + std::to_string(_ls_step) + "_sol.json");
  }

  gain_t best_gain = 1;

  while (best_gain > 0) {
    // Relocate stuff
    for (const auto& s_t : s_t_pairs) {
      if (_input._vehicles[s_t.second].capacity <
          _amounts[s_t.second] + _amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() == 0) {
        continue;
      }
      auto s_rank = ls_operator::node_candidates[s_t.first];

      std::unordered_set<index_t> t_ranks;

      // Candidate for relocate: put chosen job *before* the nearest
      // "from" job in target route.
      t_ranks.insert(
        _nearest_job_rank_in_routes_from[s_t.first][s_t.second][s_rank]);

      // Candidate for relocate: put chosen job *after* the nearest
      // "to" job in target route (or at 0 in an empty route).
      auto nearest_to_rank =
        _nearest_job_rank_in_routes_to[s_t.first][s_t.second][s_rank];

      t_ranks.insert((_sol[s_t.second].size() == 0) ? 0 : nearest_to_rank + 1);

      for (const index_t t_rank : t_ranks) {
        relocate
          r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
        if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
          best_gains[s_t.first][s_t.second] = r.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<relocate>(r);
        }
      }
    }

    // Exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if ((_sol[s_t.first].size() == 0) or (_sol[s_t.second].size() == 0)) {
        continue;
      }
      auto s_rank = ls_operator::node_candidates[s_t.first];

      std::unordered_set<index_t> t_ranks;

      // Use proximity to surrounding jobs in source route.
      if (s_rank > 0) {
        // Exchange chosen job with the one in target route that is
        // the closest from previous job in source route.
        t_ranks.insert(
          _nearest_job_rank_in_routes_from[s_t.first][s_t.second][s_rank - 1]);
      }

      if (s_rank < _sol[s_t.first].size() - 1) {
        // Exchange chosen job with the one in target route that is
        // the closest to next job in source route.
        t_ranks.insert(
          _nearest_job_rank_in_routes_to[s_t.first][s_t.second][s_rank + 1]);
      }

      // Use proximity to surrounding candidates in target route.

      // Exchange chosen job with the one in target route that is
      // *before* the closest from chosen job (or at 0 if none is
      // before).
      auto nearest_from_rank =
        _nearest_job_rank_in_routes_from[s_t.first][s_t.second][s_rank];

      t_ranks.insert((nearest_from_rank == 0) ? 0 : nearest_from_rank - 1);

      // Exchange chosen job with the one in target route that is
      // *after* the closest to chosen job (or last if none is after).
      auto nearest_to_rank =
        _nearest_job_rank_in_routes_to[s_t.first][s_t.second][s_rank];

      t_ranks.insert((nearest_to_rank == _sol[s_t.second].size() - 1)
                       ? nearest_to_rank
                       : nearest_to_rank + 1);

      for (const index_t t_rank : t_ranks) {
        exchange
          r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
        if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
          best_gains[s_t.first][s_t.second] = r.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<exchange>(r);
        }
      }
    }

    // Or-opt stuff
    for (const auto& s_t : s_t_pairs) {
      if (_input._vehicles[s_t.second].capacity <
          _amounts[s_t.second] + _double_amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() < 2) {
        continue;
      }
      auto s_rank = ls_operator::edge_candidates[s_t.first];

      std::unordered_set<index_t> t_ranks;

      // Candidate for Or-opt: put chosen edge *before* the nearest
      // "from" job in target route.
      t_ranks.insert(
        _nearest_job_rank_in_routes_from[s_t.first][s_t.second][s_rank + 1]);

      // Candidate for Or-opt: put chosen edge *after* the nearest
      // "to" job in target route (or at 0 in an empty route).
      auto nearest_to_rank =
        _nearest_job_rank_in_routes_to[s_t.first][s_t.second][s_rank];

      t_ranks.insert((_sol[s_t.second].size() == 0) ? 0 : nearest_to_rank + 1);

      for (const index_t t_rank : t_ranks) {
        or_opt r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
        if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
          best_gains[s_t.first][s_t.second] = r.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<or_opt>(r);
        }
      }
    }

    // CROSS-exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if ((_sol[s_t.first].size() < 2) or (_sol[s_t.second].size() < 2)) {
        continue;
      }
      auto s_rank = ls_operator::edge_candidates[s_t.first];

      std::unordered_set<index_t> t_ranks;

      // Use proximity to surrounding jobs in source route.
      if (s_rank > 0) {
        // Exchange chosen edge with the one in target route that
        // starts with the job that is the closest from previous job
        // in source route (or last edge if we reached end of target
        // route).
        auto nearest_from_rank =
          _nearest_job_rank_in_routes_from[s_t.first][s_t.second][s_rank - 1];

        t_ranks.insert((nearest_from_rank < _sol[s_t.second].size() - 1)
                         ? nearest_from_rank
                         : _sol[s_t.second].size() - 2);
      }

      if (s_rank < _sol[s_t.first].size() - 2) {
        // Exchange chosen edge with the one in target route that ends
        // with the job that is the closest to next job in source
        // route (or first edge if we reached start of target route).
        auto nearest_to_rank =
          _nearest_job_rank_in_routes_to[s_t.first][s_t.second][s_rank + 2];

        t_ranks.insert((nearest_to_rank == 0) ? 0 : nearest_to_rank - 1);
      }

      // Use proximity to surrounding candidates in target route.

      // Exchange chosen edge with the one in target route that ends
      // with the job *before* the closest from next-to-chosen job (or
      // first edge if none is before).
      auto nearest_from_rank =
        _nearest_job_rank_in_routes_from[s_t.first][s_t.second][s_rank + 1];

      t_ranks.insert((nearest_from_rank >= 2) ? nearest_from_rank - 2 : 0);

      // Exchange chosen edge with the one in target route that starts
      // with the job *after* the closest to chosen job (or last edge
      // if none is after).
      auto nearest_to_rank =
        _nearest_job_rank_in_routes_to[s_t.first][s_t.second][s_rank];

      t_ranks.insert((nearest_to_rank < _sol[s_t.second].size() - 2)
                       ? nearest_to_rank + 1
                       : _sol[s_t.second].size() - 2);

      for (const index_t t_rank : t_ranks) {
        cross_exchange
          r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
        if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
          best_gains[s_t.first][s_t.second] = r.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<cross_exchange>(r);
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
      std::cout << "* Best operator choice " << std::endl;

      assert(best_ops[best_source][best_target] != nullptr);

      best_ops[best_source][best_target]->log();

      best_ops[best_source][best_target]->apply();

      if (_log) {
        write_to_json(_input.format_solution(_sol),
                      false,
                      "ls_log_" + std::to_string(++_ls_step) + "_sol.json");
      }

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

      update_nearest_job_rank_in_routes(best_source, best_target);
      update_nearest_job_rank_in_routes(best_target, best_source);

      for (unsigned v = 0; v < V; ++v) {
        if (v == best_source or v == best_target) {
          continue;
        }
        s_t_pairs.emplace_back(best_source, v);
        s_t_pairs.emplace_back(v, best_source);
        best_gains[v][best_source] = 0;
        best_gains[best_source][v] = 0;
        update_nearest_job_rank_in_routes(best_source, v);
        update_nearest_job_rank_in_routes(v, best_source);

        s_t_pairs.emplace_back(best_target, v);
        s_t_pairs.emplace_back(v, best_target);
        best_gains[v][best_target] = 0;
        best_gains[best_target][v] = 0;
        update_nearest_job_rank_in_routes(best_target, v);
        update_nearest_job_rank_in_routes(v, best_target);
      }
    }
  }
}

void cvrp_local_search::run_with_fixed_source() {
  std::cout << "Running CVRP local search with fixed source." << std::endl;

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
      if (_input._vehicles[s_t.second].capacity <
          _amounts[s_t.second] + _amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() == 0) {
        continue;
      }
      auto s_rank = ls_operator::node_candidates[s_t.first];
      for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
        relocate
          r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
        if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
          best_gains[s_t.first][s_t.second] = r.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<relocate>(r);
        }
      }
    }

    // Exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if ((_sol[s_t.first].size() == 0) or (_sol[s_t.second].size() == 0)) {
        continue;
      }
      auto s_rank = ls_operator::node_candidates[s_t.first];
      for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size(); ++t_rank) {
        exchange
          r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
        if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
          best_gains[s_t.first][s_t.second] = r.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<exchange>(r);
        }
      }
    }

    // Or-opt stuff
    for (const auto& s_t : s_t_pairs) {
      if (_input._vehicles[s_t.second].capacity <
          _amounts[s_t.second] + _double_amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() < 2) {
        continue;
      }
      auto s_rank = ls_operator::edge_candidates[s_t.first];
      for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
        or_opt r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
        if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
          best_gains[s_t.first][s_t.second] = r.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<or_opt>(r);
        }
      }
    }

    // CROSS-exchange stuff
    for (const auto& s_t : s_t_pairs) {
      if ((_sol[s_t.first].size() < 2) or (_sol[s_t.second].size() < 2)) {
        continue;
      }
      auto s_rank = ls_operator::edge_candidates[s_t.first];
      for (unsigned t_rank = 0; t_rank < _sol[s_t.second].size() - 1;
           ++t_rank) {
        cross_exchange
          r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
        if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
          best_gains[s_t.first][s_t.second] = r.gain();
          best_ops[s_t.first][s_t.second] = std::make_unique<cross_exchange>(r);
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
      std::cout << "* Best operator choice " << std::endl;

      assert(best_ops[best_source][best_target] != nullptr);

      best_ops[best_source][best_target]->log();

      best_ops[best_source][best_target]->apply();

      if (_log) {
        write_to_json(_input.format_solution(_sol),
                      false,
                      "ls_log_" + std::to_string(++_ls_step) + "_sol.json");
      }

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

void cvrp_local_search::run_exhaustive_search() {
  std::cout << "Running CVRP local search exhaustively." << std::endl;

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
      if (_input._vehicles[s_t.second].capacity <
          _amounts[s_t.second] + _amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() == 0) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size(); ++s_rank) {
        for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
          relocate
            r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
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
            r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
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
          _amounts[s_t.second] + _double_amount_lower_bound) {
        // Don't try to put things in a full vehicle.
        continue;
      }
      if (_sol[s_t.first].size() < 2) {
        continue;
      }
      for (unsigned s_rank = 0; s_rank < _sol[s_t.first].size() - 1; ++s_rank) {
        for (unsigned t_rank = 0; t_rank <= _sol[s_t.second].size(); ++t_rank) {
          or_opt
            r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
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
            r(_input, _sol, _amounts, s_t.first, s_rank, s_t.second, t_rank);
          if (r.is_valid() and r.gain() > best_gains[s_t.first][s_t.second]) {
            best_gains[s_t.first][s_t.second] = r.gain();
            best_ops[s_t.first][s_t.second] =
              std::make_unique<cross_exchange>(r);
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
      std::cout << "* Best operator choice " << std::endl;

      assert(best_ops[best_source][best_target] != nullptr);

      best_ops[best_source][best_target]->log();

      best_ops[best_source][best_target]->apply();

      if (_log) {
        write_to_json(_input.format_solution(_sol),
                      false,
                      "ls_log_" + std::to_string(++_ls_step) + "_sol.json");
      }

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

void cvrp_local_search::run() {
  run_with_fixed_source_and_target();

  run_with_fixed_source();

  run_exhaustive_search();
}
