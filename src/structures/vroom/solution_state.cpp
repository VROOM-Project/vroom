/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution_state.h"

solution_state::solution_state(const input& input, const raw_solution& sol)
  : _input(input),
    _m(_input.get_matrix()),
    _V(_input._vehicles.size()),
    _empty_amount(_input.amount_size()),
    fwd_amounts(_V),
    bwd_amounts(_V),
    fwd_costs(_V),
    bwd_costs(_V),
    fwd_skill_rank(_V, std::vector<index_t>(_V)),
    bwd_skill_rank(_V, std::vector<index_t>(_V)),
    edge_costs_around_node(_V),
    node_gains(_V),
    node_candidates(_V),
    edge_costs_around_edge(_V),
    edge_gains(_V),
    edge_candidates(_V),
    nearest_job_rank_in_routes_from(_V, std::vector<std::vector<index_t>>(_V)),
    nearest_job_rank_in_routes_to(_V, std::vector<std::vector<index_t>>(_V)),
    route_costs(_V) {
  setup(sol);

  // Initialize unassigned jobs.
  index_t x = 0;
  std::generate_n(std::inserter(unassigned, unassigned.end()),
                  _input._jobs.size(),
                  [&] { return x++; });

  for (const auto& s : sol) {
    for (const auto i : s) {
      unassigned.erase(i);
    }
  }
}

void solution_state::setup(const raw_solution& sol) {
  for (std::size_t v = 0; v < _V; ++v) {
    update_amounts(sol, v);
    update_costs(sol, v);
    update_skills(sol, v);
    set_node_gains(sol, v);
    set_edge_gains(sol, v);
    update_route_cost(sol, v);
  }
}

void solution_state::update_amounts(const raw_solution& sol, index_t v) {
  fwd_amounts[v] = std::vector<amount_t>(sol[v].size());
  bwd_amounts[v] = std::vector<amount_t>(sol[v].size());
  amount_t current_amount(_input.amount_size());

  for (std::size_t i = 0; i < sol[v].size(); ++i) {
    current_amount += _input._jobs[sol[v][i]].amount;
    fwd_amounts[v][i] = current_amount;
  }

  std::transform(fwd_amounts[v].cbegin(),
                 fwd_amounts[v].cend(),
                 bwd_amounts[v].begin(),
                 [&](const auto& a) {
                   const auto& total_amount = fwd_amounts[v].back();
                   return total_amount - a;
                 });
}

void solution_state::update_costs(const raw_solution& sol, index_t v) {
  fwd_costs[v] = std::vector<cost_t>(sol[v].size());
  bwd_costs[v] = std::vector<cost_t>(sol[v].size());

  cost_t current_fwd = 0;
  cost_t current_bwd = 0;

  index_t previous_index = 0; // dummy init
  if (!sol[v].empty()) {
    previous_index = _input._jobs[sol[v][0]].index();
    fwd_costs[v][0] = current_fwd;
    bwd_costs[v][0] = current_bwd;
  }

  for (std::size_t i = 1; i < sol[v].size(); ++i) {
    auto current_index = _input._jobs[sol[v][i]].index();
    current_fwd += _m[previous_index][current_index];
    current_bwd += _m[current_index][previous_index];
    fwd_costs[v][i] = current_fwd;
    bwd_costs[v][i] = current_bwd;
    previous_index = current_index;
  }
}

void solution_state::update_skills(const raw_solution& sol, index_t v1) {
  for (std::size_t v2 = 0; v2 < _V; ++v2) {
    if (v1 == v2) {
      continue;
    }

    auto fwd =
      std::find_if_not(sol[v1].begin(), sol[v1].end(), [&](auto j_rank) {
        return _input.vehicle_ok_with_job(v2, j_rank);
      });
    fwd_skill_rank[v1][v2] = std::distance(sol[v1].begin(), fwd);

    auto bwd =
      std::find_if_not(sol[v1].rbegin(), sol[v1].rend(), [&](auto j_rank) {
        return _input.vehicle_ok_with_job(v2, j_rank);
      });
    bwd_skill_rank[v1][v2] =
      sol[v1].size() - std::distance(sol[v1].rbegin(), bwd);
  }
}

void solution_state::set_node_gains(const raw_solution& sol, index_t v) {
  node_gains[v] = std::vector<gain_t>(sol[v].size());
  edge_costs_around_node[v] = std::vector<gain_t>(sol[v].size());

  if (sol[v].size() == 0) {
    return;
  }

  // Handling first job is special due to potential open tours.
  index_t p_index;
  index_t c_index = _input._jobs[sol[v][0]].index();
  index_t n_index;

  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t new_edge_cost = 0;

  if (_input._vehicles[v].has_start()) {
    // There is a previous step before job at rank 0.
    p_index = _input._vehicles[v].start.get().index();
    previous_cost = _m[p_index][c_index];

    // Update next_cost with next job or end.
    if (sol[v].size() > 1) {
      n_index = _input._jobs[sol[v][1]].index();
      next_cost = _m[c_index][n_index];
      new_edge_cost = _m[p_index][n_index];
    } else {
      // sol[v].size() is 1 and first job is also the last.
      if (_input._vehicles[v].has_end()) {
        next_cost = _m[c_index][_input._vehicles[v].end.get().index()];
      }
    }
  } else {
    // There is a next cost either to next job or to end of route, but
    // no new edge.
    if (sol[v].size() > 1) {
      n_index = _input._jobs[sol[v][1]].index();
    } else {
      assert(_input._vehicles[v].has_end());
      n_index = _input._vehicles[v].end.get().index();
    }
    next_cost = _m[c_index][n_index];
  }

  gain_t edges_costs_around = previous_cost + next_cost;
  edge_costs_around_node[v][0] = edges_costs_around;

  gain_t current_gain = edges_costs_around - new_edge_cost;
  node_gains[v][0] = current_gain;
  gain_t best_gain = current_gain;
  node_candidates[v] = 0;

  if (sol[v].size() == 1) {
    // No more jobs.
    return;
  }

  // Handle jobs that always have a previous and next job.
  for (std::size_t i = 1; i < sol[v].size() - 1; ++i) {
    // Compute potential gain to relocate current job.
    p_index = _input._jobs[sol[v][i - 1]].index();
    c_index = _input._jobs[sol[v][i]].index();
    n_index = _input._jobs[sol[v][i + 1]].index();

    edges_costs_around = _m[p_index][c_index] + _m[c_index][n_index];
    edge_costs_around_node[v][i] = edges_costs_around;

    current_gain = edges_costs_around - _m[p_index][n_index];
    node_gains[v][i] = current_gain;

    if (current_gain > best_gain) {
      best_gain = current_gain;
      node_candidates[v] = i;
    }
  }

  // Handling last job is special due to potential open tours.
  auto last_rank = sol[v].size() - 1;
  c_index = _input._jobs[sol[v][last_rank]].index();

  previous_cost = 0;
  next_cost = 0;
  new_edge_cost = 0;

  if (_input._vehicles[v].has_end()) {
    // There is a next step after last job.
    n_index = _input._vehicles[v].end.get().index();
    next_cost = _m[c_index][n_index];

    if (sol[v].size() > 1) {
      p_index = _input._jobs[sol[v][last_rank - 1]].index();
      previous_cost = _m[p_index][c_index];
      new_edge_cost = _m[p_index][n_index];
    }
  } else {
    // There is a previous cost either from previous job or from start
    // of route, but no new edge.
    if (sol[v].size() > 1) {
      p_index = _input._jobs[sol[v][last_rank - 1]].index();
    } else {
      assert(_input._vehicles[v].has_start());
      p_index = _input._vehicles[v].start.get().index();
    }
    previous_cost = _m[p_index][c_index];
  }

  edges_costs_around = previous_cost + next_cost;
  edge_costs_around_node[v][last_rank] = edges_costs_around;

  current_gain = edges_costs_around - new_edge_cost;
  node_gains[v][last_rank] = current_gain;

  if (current_gain > best_gain) {
    node_candidates[v] = last_rank;
  }
}

void solution_state::set_edge_gains(const raw_solution& sol, index_t v) {
  std::size_t nb_edges = (sol[v].size() < 2) ? 0 : sol[v].size() - 1;

  edge_gains[v] = std::vector<gain_t>(nb_edges);
  edge_costs_around_edge[v] = std::vector<gain_t>(nb_edges);

  if (sol[v].size() < 2) {
    return;
  }

  // Handling first edge is special due to potential open tours.
  index_t p_index;
  index_t c_index = _input._jobs[sol[v][0]].index();
  index_t after_c_index = _input._jobs[sol[v][1]].index();
  index_t n_index;

  gain_t previous_cost = 0;
  gain_t next_cost = 0;
  gain_t new_edge_cost = 0;

  if (_input._vehicles[v].has_start()) {
    // There is a previous step before job at rank 0.
    p_index = _input._vehicles[v].start.get().index();
    previous_cost = _m[p_index][c_index];

    // Update next_cost with next job or end.
    if (sol[v].size() > 2) {
      n_index = _input._jobs[sol[v][2]].index();
      next_cost = _m[after_c_index][n_index];
      new_edge_cost = _m[p_index][n_index];
    } else {
      // sol[v].size() is 2 and first edge is also the last.
      if (_input._vehicles[v].has_end()) {
        next_cost = _m[after_c_index][_input._vehicles[v].end.get().index()];
      }
    }
  } else {
    // There is a next cost either to next job or to end of route, but
    // no new edge.
    if (sol[v].size() > 2) {
      n_index = _input._jobs[sol[v][2]].index();
    } else {
      assert(_input._vehicles[v].has_end());
      n_index = _input._vehicles[v].end.get().index();
    }
    next_cost = _m[after_c_index][n_index];
  }

  gain_t edges_costs_around = previous_cost + next_cost;
  edge_costs_around_edge[v][0] = edges_costs_around;

  gain_t current_gain = edges_costs_around - new_edge_cost;
  edge_gains[v][0] = current_gain;
  gain_t best_gain = current_gain;
  edge_candidates[v] = 0;

  if (sol[v].size() == 2) {
    // No more edges.
    return;
  }

  // Handle edges that always have a previous and next job.
  for (std::size_t i = 1; i < nb_edges - 1; ++i) {
    // Compute potential gain to relocate edge from current to next
    // job.
    p_index = _input._jobs[sol[v][i - 1]].index();
    c_index = _input._jobs[sol[v][i]].index();
    after_c_index = _input._jobs[sol[v][i + 1]].index();
    n_index = _input._jobs[sol[v][i + 2]].index();

    edges_costs_around = _m[p_index][c_index] + _m[after_c_index][n_index];
    edge_costs_around_edge[v][i] = edges_costs_around;

    current_gain = edges_costs_around - _m[p_index][n_index];
    edge_gains[v][i] = current_gain;

    if (current_gain > best_gain) {
      best_gain = current_gain;
      edge_candidates[v] = i;
    }
  }

  // Handling last edge is special due to potential open tours.
  auto last_edge_rank = nb_edges - 1;
  c_index = _input._jobs[sol[v][last_edge_rank]].index();
  after_c_index = _input._jobs[sol[v][last_edge_rank + 1]].index();

  previous_cost = 0;
  next_cost = 0;
  new_edge_cost = 0;

  if (_input._vehicles[v].has_end()) {
    // There is a next step after last job.
    n_index = _input._vehicles[v].end.get().index();
    next_cost = _m[after_c_index][n_index];

    if (sol[v].size() > 2) {
      p_index = _input._jobs[sol[v][last_edge_rank - 1]].index();
      previous_cost = _m[p_index][c_index];
      new_edge_cost = _m[p_index][n_index];
    }
  } else {
    // There is a previous cost either from previous job or from start
    // of route, but no new edge.
    if (sol[v].size() > 2) {
      p_index = _input._jobs[sol[v][last_edge_rank - 1]].index();
    } else {
      assert(_input._vehicles[v].has_start());
      p_index = _input._vehicles[v].start.get().index();
    }
    previous_cost = _m[p_index][c_index];
  }

  edges_costs_around = previous_cost + next_cost;
  edge_costs_around_edge[v][last_edge_rank] = edges_costs_around;

  current_gain = edges_costs_around - new_edge_cost;
  edge_gains[v][last_edge_rank] = current_gain;

  if (current_gain > best_gain) {
    edge_candidates[v] = last_edge_rank;
  }
}

void solution_state::update_nearest_job_rank_in_routes(const raw_solution& sol,
                                                       index_t v1,
                                                       index_t v2) {
  nearest_job_rank_in_routes_from[v1][v2] =
    std::vector<index_t>(sol[v1].size());
  nearest_job_rank_in_routes_to[v1][v2] = std::vector<index_t>(sol[v1].size());

  for (std::size_t r1 = 0; r1 < sol[v1].size(); ++r1) {
    index_t index_r1 = _input._jobs[sol[v1][r1]].index();

    auto min_from = std::numeric_limits<cost_t>::max();
    auto min_to = std::numeric_limits<cost_t>::max();
    index_t best_from_rank = 0;
    index_t best_to_rank = 0;

    for (std::size_t r2 = 0; r2 < sol[v2].size(); ++r2) {
      index_t index_r2 = _input._jobs[sol[v2][r2]].index();
      if (_m[index_r1][index_r2] < min_from) {
        min_from = _m[index_r1][index_r2];
        best_from_rank = r2;
      }
      if (_m[index_r2][index_r1] < min_to) {
        min_to = _m[index_r2][index_r1];
        best_to_rank = r2;
      }
    }

    nearest_job_rank_in_routes_from[v1][v2][r1] = best_from_rank;
    nearest_job_rank_in_routes_to[v1][v2][r1] = best_to_rank;
  }
}

cost_t solution_state::route_cost_for_vehicle(
  index_t vehicle_rank,
  const std::vector<index_t>& route) const {
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

void solution_state::update_route_cost(const raw_solution& sol, index_t v) {
  route_costs[v] = route_cost_for_vehicle(v, sol[v]);
}

const amount_t& solution_state::total_amount(index_t v) const {
  if (!fwd_amounts[v].empty()) {
    return fwd_amounts[v].back();
  } else {
    return _empty_amount;
  }
}
