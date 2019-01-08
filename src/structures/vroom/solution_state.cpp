/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>

#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

namespace vroom {
namespace utils {

SolutionState::SolutionState(const Input& input)
  : _input(input),
    _m(_input.get_matrix()),
    _nb_vehicles(_input.vehicles.size()),
    _empty_amount(_input.amount_size()),
    fwd_amounts(_nb_vehicles),
    bwd_amounts(_nb_vehicles),
    fwd_costs(_nb_vehicles),
    bwd_costs(_nb_vehicles),
    fwd_skill_rank(_nb_vehicles, std::vector<Index>(_nb_vehicles)),
    bwd_skill_rank(_nb_vehicles, std::vector<Index>(_nb_vehicles)),
    edge_costs_around_node(_nb_vehicles),
    node_gains(_nb_vehicles),
    node_candidates(_nb_vehicles),
    edge_costs_around_edge(_nb_vehicles),
    edge_gains(_nb_vehicles),
    edge_candidates(_nb_vehicles),
    nearest_job_rank_in_routes_from(_nb_vehicles,
                                    std::vector<std::vector<Index>>(
                                      _nb_vehicles)),
    nearest_job_rank_in_routes_to(_nb_vehicles,
                                  std::vector<std::vector<Index>>(
                                    _nb_vehicles)),
    route_costs(_nb_vehicles) {
}

void SolutionState::setup(const std::vector<Index>& r, Index v) {
  update_amounts(r, v);
  update_costs(r, v);
  update_skills(r, v);
  set_node_gains(r, v);
  set_edge_gains(r, v);
#ifndef NDEBUG
  update_route_cost(r, v);
#endif
}

void SolutionState::setup(const RawSolution& sol) {
  for (std::size_t v = 0; v < _nb_vehicles; ++v) {
    setup(sol[v].route, v);
  }

  // Initialize unassigned jobs.
  Index x = 0;
  std::generate_n(std::inserter(unassigned, unassigned.end()),
                  _input.jobs.size(),
                  [&] { return x++; });

  for (const auto& s : sol) {
    for (const auto i : s.route) {
      unassigned.erase(i);
    }
  }
}

void SolutionState::setup(const TWSolution& tw_sol) {
  for (std::size_t v = 0; v < _nb_vehicles; ++v) {
    setup(tw_sol[v].route, v);
  }

  // Initialize unassigned jobs.
  Index x = 0;
  std::generate_n(std::inserter(unassigned, unassigned.end()),
                  _input.jobs.size(),
                  [&] { return x++; });

  for (const auto& tw_r : tw_sol) {
    for (const auto i : tw_r.route) {
      unassigned.erase(i);
    }
  }
}

void SolutionState::update_amounts(const std::vector<Index>& route, Index v) {
  fwd_amounts[v] = std::vector<Amount>(route.size());
  bwd_amounts[v] = std::vector<Amount>(route.size());
  Amount current_amount(_input.amount_size());

  for (std::size_t i = 0; i < route.size(); ++i) {
    current_amount += _input.jobs[route[i]].amount;
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

void SolutionState::update_costs(const std::vector<Index>& route, Index v) {
  fwd_costs[v] = std::vector<Cost>(route.size());
  bwd_costs[v] = std::vector<Cost>(route.size());

  Cost current_fwd = 0;
  Cost current_bwd = 0;

  Index previous_index = 0; // dummy init
  if (!route.empty()) {
    previous_index = _input.jobs[route[0]].index();
    fwd_costs[v][0] = current_fwd;
    bwd_costs[v][0] = current_bwd;
  }

  for (std::size_t i = 1; i < route.size(); ++i) {
    auto current_index = _input.jobs[route[i]].index();
    current_fwd += _m[previous_index][current_index];
    current_bwd += _m[current_index][previous_index];
    fwd_costs[v][i] = current_fwd;
    bwd_costs[v][i] = current_bwd;
    previous_index = current_index;
  }
}

void SolutionState::update_skills(const std::vector<Index>& route, Index v1) {
  for (std::size_t v2 = 0; v2 < _nb_vehicles; ++v2) {
    if (v1 == v2) {
      continue;
    }

    auto fwd = std::find_if_not(route.begin(), route.end(), [&](auto j_rank) {
      return _input.vehicle_ok_with_job(v2, j_rank);
    });
    fwd_skill_rank[v1][v2] = std::distance(route.begin(), fwd);

    auto bwd = std::find_if_not(route.rbegin(), route.rend(), [&](auto j_rank) {
      return _input.vehicle_ok_with_job(v2, j_rank);
    });
    bwd_skill_rank[v1][v2] = route.size() - std::distance(route.rbegin(), bwd);
  }
}

void SolutionState::set_node_gains(const std::vector<Index>& route, Index v) {
  node_gains[v] = std::vector<Gain>(route.size());
  edge_costs_around_node[v] = std::vector<Gain>(route.size());

  if (route.size() == 0) {
    return;
  }

  // Handling first job is special due to potential open tours.
  Index p_index;
  Index c_index = _input.jobs[route[0]].index();
  Index n_index;

  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain new_edge_cost = 0;

  if (_input.vehicles[v].has_start()) {
    // There is a previous step before job at rank 0.
    p_index = _input.vehicles[v].start.get().index();
    previous_cost = _m[p_index][c_index];

    // Update next_cost with next job or end.
    if (route.size() > 1) {
      n_index = _input.jobs[route[1]].index();
      next_cost = _m[c_index][n_index];
      new_edge_cost = _m[p_index][n_index];
    } else {
      // route.size() is 1 and first job is also the last.
      if (_input.vehicles[v].has_end()) {
        next_cost = _m[c_index][_input.vehicles[v].end.get().index()];
      }
    }
  } else {
    // There is a next cost either to next job or to end of route, but
    // no new edge.
    if (route.size() > 1) {
      n_index = _input.jobs[route[1]].index();
    } else {
      assert(_input.vehicles[v].has_end());
      n_index = _input.vehicles[v].end.get().index();
    }
    next_cost = _m[c_index][n_index];
  }

  Gain edges_costs_around = previous_cost + next_cost;
  edge_costs_around_node[v][0] = edges_costs_around;

  Gain current_gain = edges_costs_around - new_edge_cost;
  node_gains[v][0] = current_gain;
  Gain best_gain = current_gain;
  node_candidates[v] = 0;

  if (route.size() == 1) {
    // No more jobs.
    return;
  }

  // Handle jobs that always have a previous and next job.
  for (std::size_t i = 1; i < route.size() - 1; ++i) {
    // Compute potential gain to relocate current job.
    p_index = _input.jobs[route[i - 1]].index();
    c_index = _input.jobs[route[i]].index();
    n_index = _input.jobs[route[i + 1]].index();

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
  auto last_rank = route.size() - 1;
  c_index = _input.jobs[route[last_rank]].index();

  previous_cost = 0;
  next_cost = 0;
  new_edge_cost = 0;

  if (_input.vehicles[v].has_end()) {
    // There is a next step after last job.
    n_index = _input.vehicles[v].end.get().index();
    next_cost = _m[c_index][n_index];

    if (route.size() > 1) {
      p_index = _input.jobs[route[last_rank - 1]].index();
      previous_cost = _m[p_index][c_index];
      new_edge_cost = _m[p_index][n_index];
    }
  } else {
    // There is a previous cost either from previous job or from start
    // of route, but no new edge.
    if (route.size() > 1) {
      p_index = _input.jobs[route[last_rank - 1]].index();
    } else {
      assert(_input.vehicles[v].has_start());
      p_index = _input.vehicles[v].start.get().index();
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

void SolutionState::set_edge_gains(const std::vector<Index>& route, Index v) {
  std::size_t nb_edges = (route.size() < 2) ? 0 : route.size() - 1;

  edge_gains[v] = std::vector<Gain>(nb_edges);
  edge_costs_around_edge[v] = std::vector<Gain>(nb_edges);

  if (route.size() < 2) {
    return;
  }

  // Handling first edge is special due to potential open tours.
  Index p_index;
  Index c_index = _input.jobs[route[0]].index();
  Index after_c_index = _input.jobs[route[1]].index();
  Index n_index;

  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain new_edge_cost = 0;

  if (_input.vehicles[v].has_start()) {
    // There is a previous step before job at rank 0.
    p_index = _input.vehicles[v].start.get().index();
    previous_cost = _m[p_index][c_index];

    // Update next_cost with next job or end.
    if (route.size() > 2) {
      n_index = _input.jobs[route[2]].index();
      next_cost = _m[after_c_index][n_index];
      new_edge_cost = _m[p_index][n_index];
    } else {
      // route.size() is 2 and first edge is also the last.
      if (_input.vehicles[v].has_end()) {
        next_cost = _m[after_c_index][_input.vehicles[v].end.get().index()];
      }
    }
  } else {
    // There is a next cost either to next job or to end of route, but
    // no new edge.
    if (route.size() > 2) {
      n_index = _input.jobs[route[2]].index();
    } else {
      assert(_input.vehicles[v].has_end());
      n_index = _input.vehicles[v].end.get().index();
    }
    next_cost = _m[after_c_index][n_index];
  }

  Gain edges_costs_around = previous_cost + next_cost;
  edge_costs_around_edge[v][0] = edges_costs_around;

  Gain current_gain = edges_costs_around - new_edge_cost;
  edge_gains[v][0] = current_gain;
  Gain best_gain = current_gain;
  edge_candidates[v] = 0;

  if (route.size() == 2) {
    // No more edges.
    return;
  }

  // Handle edges that always have a previous and next job.
  for (std::size_t i = 1; i < nb_edges - 1; ++i) {
    // Compute potential gain to relocate edge from current to next
    // job.
    p_index = _input.jobs[route[i - 1]].index();
    c_index = _input.jobs[route[i]].index();
    after_c_index = _input.jobs[route[i + 1]].index();
    n_index = _input.jobs[route[i + 2]].index();

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
  c_index = _input.jobs[route[last_edge_rank]].index();
  after_c_index = _input.jobs[route[last_edge_rank + 1]].index();

  previous_cost = 0;
  next_cost = 0;
  new_edge_cost = 0;

  if (_input.vehicles[v].has_end()) {
    // There is a next step after last job.
    n_index = _input.vehicles[v].end.get().index();
    next_cost = _m[after_c_index][n_index];

    if (route.size() > 2) {
      p_index = _input.jobs[route[last_edge_rank - 1]].index();
      previous_cost = _m[p_index][c_index];
      new_edge_cost = _m[p_index][n_index];
    }
  } else {
    // There is a previous cost either from previous job or from start
    // of route, but no new edge.
    if (route.size() > 2) {
      p_index = _input.jobs[route[last_edge_rank - 1]].index();
    } else {
      assert(_input.vehicles[v].has_start());
      p_index = _input.vehicles[v].start.get().index();
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

void SolutionState::update_nearest_job_rank_in_routes(
  const std::vector<Index>& route_1,
  const std::vector<Index>& route_2,
  Index v1,
  Index v2) {
  nearest_job_rank_in_routes_from[v1][v2] = std::vector<Index>(route_1.size());
  nearest_job_rank_in_routes_to[v1][v2] = std::vector<Index>(route_1.size());

  for (std::size_t r1 = 0; r1 < route_1.size(); ++r1) {
    Index index_r1 = _input.jobs[route_1[r1]].index();

    auto min_from = std::numeric_limits<Cost>::max();
    auto min_to = std::numeric_limits<Cost>::max();
    Index best_from_rank = 0;
    Index best_to_rank = 0;

    for (std::size_t r2 = 0; r2 < route_2.size(); ++r2) {
      Index index_r2 = _input.jobs[route_2[r2]].index();
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

void SolutionState::update_route_cost(const std::vector<Index>& route,
                                      Index v) {
  route_costs[v] = route_cost_for_vehicle(_input, v, route);
}

const Amount& SolutionState::total_amount(Index v) const {
  if (!fwd_amounts[v].empty()) {
    return fwd_amounts[v].back();
  } else {
    return _empty_amount;
  }
}

} // namespace utils
} // namespace vroom
