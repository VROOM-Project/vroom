/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>

#include "structures/vroom/solution_state.h"
#include "utils/helpers.h"

namespace vroom::utils {

SolutionState::SolutionState(const Input& input)
  : _input(input),
    _nb_vehicles(_input.vehicles.size()),
    fwd_costs(_nb_vehicles, std::vector<std::vector<Eval>>(_nb_vehicles)),
    bwd_costs(_nb_vehicles, std::vector<std::vector<Eval>>(_nb_vehicles)),
    fwd_skill_rank(_nb_vehicles, std::vector<Index>(_nb_vehicles)),
    bwd_skill_rank(_nb_vehicles, std::vector<Index>(_nb_vehicles)),
    fwd_priority(_nb_vehicles),
    bwd_priority(_nb_vehicles),
    edge_evals_around_node(_nb_vehicles),
    node_gains(_nb_vehicles),
    node_candidates(_nb_vehicles),
    edge_evals_around_edge(_nb_vehicles),
    edge_gains(_nb_vehicles),
    edge_candidates(_nb_vehicles),
    pd_gains(_nb_vehicles),
    matching_delivery_rank(_nb_vehicles),
    matching_pickup_rank(_nb_vehicles),
    cheapest_job_rank_in_routes_from(_nb_vehicles,
                                     std::vector<std::vector<Index>>(
                                       _nb_vehicles)),
    cheapest_job_rank_in_routes_to(_nb_vehicles,
                                   std::vector<std::vector<Index>>(
                                     _nb_vehicles)),
    insertion_ranks_begin(_nb_vehicles),
    insertion_ranks_end(_nb_vehicles),
    weak_insertion_ranks_begin(_nb_vehicles),
    weak_insertion_ranks_end(_nb_vehicles),
    route_evals(_nb_vehicles),
    route_bbox(_nb_vehicles, BBox()) {
}

template <class Route> void SolutionState::setup(const Route& r, Index v) {
  update_costs(r.route, v);
  update_skills(r.route, v);
  update_priorities(r.route, v);
  set_node_gains(r.route, v);
  set_edge_gains(r.route, v);
  set_pd_matching_ranks(r.route, v);
  set_pd_gains(r.route, v);
  set_insertion_ranks(r, v);
  update_route_eval(r.route, v);
  update_route_bbox(r.route, v);
}

template <class Solution> void SolutionState::setup(const Solution& sol) {
  for (std::size_t v = 0; v < _nb_vehicles; ++v) {
    setup(sol[v], v);
  }

  // Initialize unassigned jobs.
  Index x = 0;
  std::generate_n(std::inserter(unassigned, unassigned.end()),
                  _input.jobs.size(),
                  [&] { return x++; });

  for (const auto& r : sol) {
    for (const auto i : r.route) {
      unassigned.erase(i);
    }
  }
}

void SolutionState::update_costs(const std::vector<Index>& route, Index v) {
  fwd_costs[v] =
    std::vector<std::vector<Eval>>(_nb_vehicles,
                                   std::vector<Eval>(route.size()));
  bwd_costs[v] =
    std::vector<std::vector<Eval>>(_nb_vehicles,
                                   std::vector<Eval>(route.size()));

  Index previous_index = 0; // dummy init
  if (!route.empty()) {
    previous_index = _input.jobs[route[0]].index();
    for (Index v_rank = 0; v_rank < _nb_vehicles; ++v_rank) {
      fwd_costs[v][v_rank][0] = Eval();
      bwd_costs[v][v_rank][0] = Eval();
    }
  }

  for (std::size_t i = 1; i < route.size(); ++i) {
    const auto current_index = _input.jobs[route[i]].index();
    for (Index v_rank = 0; v_rank < _nb_vehicles; ++v_rank) {
      const auto& other_v = _input.vehicles[v_rank];
      fwd_costs[v][v_rank][i] = fwd_costs[v][v_rank][i - 1] +
                                other_v.eval(previous_index, current_index);

      bwd_costs[v][v_rank][i] = bwd_costs[v][v_rank][i - 1] +
                                other_v.eval(current_index, previous_index);
    }
    previous_index = current_index;
  }
}

void SolutionState::update_skills(const std::vector<Index>& route, Index v1) {
  for (std::size_t v2 = 0; v2 < _nb_vehicles; ++v2) {
    if (v1 == v2) {
      continue;
    }

    auto fwd = std::ranges::find_if_not(route, [&](auto j_rank) {
      return _input.vehicle_ok_with_job(v2, j_rank);
    });
    fwd_skill_rank[v1][v2] = std::distance(route.begin(), fwd);

    auto bwd = std::find_if_not(route.rbegin(), route.rend(), [&](auto j_rank) {
      return _input.vehicle_ok_with_job(v2, j_rank);
    });
    bwd_skill_rank[v1][v2] = route.size() - std::distance(route.rbegin(), bwd);
  }
}

void SolutionState::update_priorities(const std::vector<Index>& route,
                                      Index v) {
  fwd_priority[v].resize(route.size());
  std::inclusive_scan(
    route.cbegin(),
    route.cend(),
    fwd_priority[v].begin(),
    [this](const auto p, const auto j) { return p + _input.jobs[j].priority; },
    0);

  bwd_priority[v].resize(route.size());
  std::inclusive_scan(
    route.crbegin(),
    route.crend(),
    bwd_priority[v].rbegin(),
    [this](const auto p, const auto j) { return p + _input.jobs[j].priority; },
    0);
}

void SolutionState::set_node_gains(const std::vector<Index>& route, Index v) {
  node_gains[v] = std::vector<Eval>(route.size());
  edge_evals_around_node[v] = std::vector<Eval>(route.size());

  if (route.empty()) {
    return;
  }

  // Handling first job is special due to potential open tours.
  Index p_index;
  Index c_index = _input.jobs[route[0]].index();
  Index n_index;

  Eval previous_eval;
  Eval next_eval;
  Eval new_edge_eval;

  const auto& vehicle = _input.vehicles[v];
  if (vehicle.has_start()) {
    // There is a previous step before job at rank 0.
    p_index = vehicle.start.value().index();
    previous_eval = vehicle.eval(p_index, c_index);

    // Update next_eval with next job or end.
    if (route.size() > 1) {
      n_index = _input.jobs[route[1]].index();
      next_eval = vehicle.eval(c_index, n_index);
      new_edge_eval = vehicle.eval(p_index, n_index);
    } else {
      // route.size() is 1 and first job is also the last.
      if (vehicle.has_end()) {
        next_eval = vehicle.eval(c_index, vehicle.end.value().index());
      }
    }
  } else {
    // There is a next eval either to next job or to end of route, but
    // no new edge.
    if (route.size() > 1) {
      n_index = _input.jobs[route[1]].index();
    } else {
      assert(vehicle.has_end());
      n_index = vehicle.end.value().index();
    }
    next_eval = vehicle.eval(c_index, n_index);
  }

  Eval edges_evals_around = previous_eval + next_eval;
  edge_evals_around_node[v][0] = edges_evals_around;

  Eval current_gain = edges_evals_around - new_edge_eval;
  node_gains[v][0] = current_gain;
  Eval best_gain = current_gain;
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

    edges_evals_around =
      vehicle.eval(p_index, c_index) + vehicle.eval(c_index, n_index);
    edge_evals_around_node[v][i] = edges_evals_around;

    current_gain = edges_evals_around - vehicle.eval(p_index, n_index);
    node_gains[v][i] = current_gain;

    if (best_gain < current_gain) {
      best_gain = current_gain;
      node_candidates[v] = i;
    }
  }

  // Handling last job is special due to potential open tours.
  auto last_rank = route.size() - 1;
  c_index = _input.jobs[route[last_rank]].index();

  previous_eval = Eval();
  next_eval = Eval();
  new_edge_eval = Eval();

  if (vehicle.has_end()) {
    // There is a next step after last job.
    n_index = vehicle.end.value().index();
    next_eval = vehicle.eval(c_index, n_index);

    if (route.size() > 1) {
      p_index = _input.jobs[route[last_rank - 1]].index();
      previous_eval = vehicle.eval(p_index, c_index);
      new_edge_eval = vehicle.eval(p_index, n_index);
    }
  } else {
    // There is a previous eval either from previous job or from start
    // of route, but no new edge.
    if (route.size() > 1) {
      p_index = _input.jobs[route[last_rank - 1]].index();
    } else {
      assert(vehicle.has_start());
      p_index = vehicle.start.value().index();
    }
    previous_eval = vehicle.eval(p_index, c_index);
  }

  edges_evals_around = previous_eval + next_eval;
  edge_evals_around_node[v][last_rank] = edges_evals_around;

  current_gain = edges_evals_around - new_edge_eval;
  node_gains[v][last_rank] = current_gain;

  if (best_gain < current_gain) {
    node_candidates[v] = last_rank;
  }
}

void SolutionState::set_edge_gains(const std::vector<Index>& route, Index v) {
  const std::size_t nb_edges = (route.size() < 2) ? 0 : route.size() - 1;

  edge_gains[v] = std::vector<Eval>(nb_edges);
  edge_evals_around_edge[v] = std::vector<Eval>(nb_edges);

  if (route.size() < 2) {
    return;
  }

  // Handling first edge is special due to potential open tours.
  Index p_index;
  Index c_index = _input.jobs[route[0]].index();
  Index after_c_index = _input.jobs[route[1]].index();
  Index n_index;

  Eval previous_eval;
  Eval next_eval;
  Eval new_edge_eval;

  const auto& vehicle = _input.vehicles[v];
  if (vehicle.has_start()) {
    // There is a previous step before job at rank 0.
    p_index = vehicle.start.value().index();
    previous_eval = vehicle.eval(p_index, c_index);

    // Update next_eval with next job or end.
    if (route.size() > 2) {
      n_index = _input.jobs[route[2]].index();
      next_eval = vehicle.eval(after_c_index, n_index);
      new_edge_eval = vehicle.eval(p_index, n_index);
    } else {
      // route.size() is 2 and first edge is also the last.
      if (vehicle.has_end()) {
        next_eval = vehicle.eval(after_c_index, vehicle.end.value().index());
      }
    }
  } else {
    // There is a next eval either to next job or to end of route, but
    // no new edge.
    if (route.size() > 2) {
      n_index = _input.jobs[route[2]].index();
    } else {
      assert(vehicle.has_end());
      n_index = vehicle.end.value().index();
    }
    next_eval = vehicle.eval(after_c_index, n_index);
  }

  Eval edges_evals_around = previous_eval + next_eval;
  edge_evals_around_edge[v][0] = edges_evals_around;

  Eval current_gain = edges_evals_around - new_edge_eval;
  edge_gains[v][0] = current_gain;
  Eval best_gain = current_gain;
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

    edges_evals_around =
      vehicle.eval(p_index, c_index) + vehicle.eval(after_c_index, n_index);
    edge_evals_around_edge[v][i] = edges_evals_around;

    current_gain = edges_evals_around - vehicle.eval(p_index, n_index);
    edge_gains[v][i] = current_gain;

    if (best_gain < current_gain) {
      best_gain = current_gain;
      edge_candidates[v] = i;
    }
  }

  // Handling last edge is special due to potential open tours.
  auto last_edge_rank = nb_edges - 1;
  c_index = _input.jobs[route[last_edge_rank]].index();
  after_c_index = _input.jobs[route[last_edge_rank + 1]].index();

  previous_eval = Eval();
  next_eval = Eval();
  new_edge_eval = Eval();

  if (vehicle.has_end()) {
    // There is a next step after last job.
    n_index = vehicle.end.value().index();
    next_eval = vehicle.eval(after_c_index, n_index);

    if (route.size() > 2) {
      p_index = _input.jobs[route[last_edge_rank - 1]].index();
      previous_eval = vehicle.eval(p_index, c_index);
      new_edge_eval = vehicle.eval(p_index, n_index);
    }
  } else {
    // There is a previous eval either from previous job or from start
    // of route, but no new edge.
    if (route.size() > 2) {
      p_index = _input.jobs[route[last_edge_rank - 1]].index();
    } else {
      assert(vehicle.has_start());
      p_index = vehicle.start.value().index();
    }
    previous_eval = vehicle.eval(p_index, c_index);
  }

  edges_evals_around = previous_eval + next_eval;
  edge_evals_around_edge[v][last_edge_rank] = edges_evals_around;

  current_gain = edges_evals_around - new_edge_eval;
  edge_gains[v][last_edge_rank] = current_gain;

  if (best_gain < current_gain) {
    edge_candidates[v] = last_edge_rank;
  }
}

void SolutionState::set_pd_gains(const std::vector<Index>& route, Index v) {
  // Expects to have valid values in node_gains, so should be run
  // after set_node_gains. Expects to have valid values in
  // matching_delivery_rank, so should be run after
  // set_pd_matching_ranks.
  pd_gains[v] = std::vector<Eval>(route.size());

  const auto& vehicle = _input.vehicles[v];

  for (std::size_t pickup_rank = 0; pickup_rank < route.size(); ++pickup_rank) {
    if (_input.jobs[route[pickup_rank]].type != JOB_TYPE::PICKUP) {
      continue;
    }
    const Index pickup_index = _input.jobs[route[pickup_rank]].index();
    const Index delivery_rank = matching_delivery_rank[v][pickup_rank];
    const Index delivery_index = _input.jobs[route[delivery_rank]].index();

    if (pickup_rank + 1 == delivery_rank) {
      // Pickup and delivery in a row.
      Eval previous_eval;
      Eval next_eval;
      Eval new_edge_eval;
      Index p_index;
      Index n_index;

      // Compute eval for step before pickup.
      bool has_previous_step = false;
      if (pickup_rank > 0) {
        has_previous_step = true;
        p_index = _input.jobs[route[pickup_rank - 1]].index();
        previous_eval = vehicle.eval(p_index, pickup_index);
      } else {
        if (vehicle.has_start()) {
          has_previous_step = true;
          p_index = vehicle.start.value().index();
          previous_eval = vehicle.eval(p_index, pickup_index);
        }
      }

      // Compute eval for step after delivery.
      bool has_next_step = false;
      if (delivery_rank < route.size() - 1) {
        has_next_step = true;
        n_index = _input.jobs[route[delivery_rank + 1]].index();
        next_eval = vehicle.eval(delivery_index, n_index);
      } else {
        if (vehicle.has_end()) {
          has_next_step = true;
          n_index = vehicle.end.value().index();
          next_eval = vehicle.eval(delivery_index, n_index);
        }
      }

      if (has_previous_step && has_next_step && (route.size() > 2)) {
        // No new edge with an open trip or if removing P&D creates an
        // empty route.
        new_edge_eval = vehicle.eval(p_index, n_index);
      }

      pd_gains[v][pickup_rank] = previous_eval +
                                 vehicle.eval(pickup_index, delivery_index) +
                                 next_eval - new_edge_eval;
    } else {
      // Simply add both gains as neighbouring edges are disjoint.
      pd_gains[v][pickup_rank] =
        node_gains[v][pickup_rank] + node_gains[v][delivery_rank];
    }
  }
}

void SolutionState::set_pd_matching_ranks(const std::vector<Index>& route,
                                          Index v) {
  matching_delivery_rank[v] = std::vector<Index>(route.size());
  matching_pickup_rank[v] = std::vector<Index>(route.size());

  std::unordered_map<Index, Index> pickup_route_rank_to_input_rank;
  std::unordered_map<Index, Index> delivery_input_rank_to_route_rank;

  for (std::size_t i = 0; i < route.size(); ++i) {
    switch (_input.jobs[route[i]].type) {
      using enum JOB_TYPE;
    case SINGLE:
      break;
    case PICKUP:
      pickup_route_rank_to_input_rank.insert({i, route[i]});
      break;
    case DELIVERY:
      delivery_input_rank_to_route_rank.insert({route[i], i});
      break;
    }
  }

  assert(pickup_route_rank_to_input_rank.size() ==
         delivery_input_rank_to_route_rank.size());
  for (const auto& [pickup_route_rank, pickup_input_rank] :
       pickup_route_rank_to_input_rank) {
    // Relies of the fact that associated pickup and delivery are
    // stored sequentially in input jobs vector.
    auto delivery_input_rank = pickup_input_rank + 1;
    auto search = delivery_input_rank_to_route_rank.find(delivery_input_rank);
    assert(search != delivery_input_rank_to_route_rank.end());
    auto delivery_route_rank = search->second;

    matching_delivery_rank[v][pickup_route_rank] = delivery_route_rank;
    matching_pickup_rank[v][delivery_route_rank] = pickup_route_rank;
  }
}

void SolutionState::set_insertion_ranks(const RawRoute& r, Index v) {
  insertion_ranks_end[v] =
    std::vector<Index>(_input.jobs.size(), r.route.size() + 1);
  insertion_ranks_begin[v] = std::vector<Index>(_input.jobs.size(), 0);

  weak_insertion_ranks_end[v] =
    std::vector<Index>(_input.jobs.size(), r.route.size() + 1);
  weak_insertion_ranks_begin[v] = std::vector<Index>(_input.jobs.size(), 0);
}

void SolutionState::set_insertion_ranks(const TWRoute& tw_r, Index v) {
  insertion_ranks_end[v] =
    std::vector<Index>(_input.jobs.size(), tw_r.route.size() + 1);
  insertion_ranks_begin[v] = std::vector<Index>(_input.jobs.size(), 0);

  weak_insertion_ranks_end[v] =
    std::vector<Index>(_input.jobs.size(), tw_r.route.size() + 1);
  weak_insertion_ranks_begin[v] = std::vector<Index>(_input.jobs.size(), 0);

  if (tw_r.empty()) {
    return;
  }

  const auto& vehicle = _input.vehicles[v];
  const auto v_type = vehicle.type;

  for (std::size_t j = 0; j < _input.jobs.size(); ++j) {
    if (!_input.vehicle_ok_with_job(v, j)) {
      insertion_ranks_end[v][j] = 0;
      continue;
    }

    const auto& job = _input.jobs[j];

    const auto job_available = job.tws.front().start;
    const auto job_deadline = job.tws.back().end;
    const auto job_index = job.index();

    // Handle insertion_ranks_*
    for (std::size_t t = 0; t < tw_r.route.size(); ++t) {
      if (tw_r.route[t] == j) {
        continue;
      }
      if (job_deadline <
          tw_r.earliest[t] + tw_r.action_time[t] +
            vehicle.duration(_input.jobs[tw_r.route[t]].index(), job_index)) {
        // Too late to perform job any time after task at t based on
        // its earliest date in route for v.
        insertion_ranks_end[v][j] = t + 1;
        break;
      }
    }
    for (std::size_t t = 0; t < tw_r.route.size(); ++t) {
      const auto rev_t = tw_r.route.size() - 1 - t;
      if (tw_r.route[rev_t] == j) {
        continue;
      }
      if (tw_r.latest[rev_t] <
          job_available + job.services[v_type] +
            vehicle.duration(job_index,
                             _input.jobs[tw_r.route[rev_t]].index())) {
        // Job is available too late to be performed any time before
        // task at rev_t based on its latest date in route for v.
        insertion_ranks_begin[v][j] = rev_t + 1;
        break;
      }
    }

    // Handle weak_insertion_ranks_*
    for (std::size_t t = 0; t < tw_r.route.size(); ++t) {
      if (tw_r.route[t] == j) {
        continue;
      }
      const auto& task = _input.jobs[tw_r.route[t]];
      if (job_deadline < task.tws.front().start + task.services[v_type] +
                           vehicle.duration(task.index(), job_index)) {
        // Too late to perform job any time after task at t solely
        // based on its TW.
        weak_insertion_ranks_end[v][j] = t + 1;
        assert(insertion_ranks_end[v][j] <= weak_insertion_ranks_end[v][j]);
        break;
      }
    }
    for (std::size_t t = 0; t < tw_r.route.size(); ++t) {
      const auto rev_t = tw_r.route.size() - 1 - t;
      if (tw_r.route[rev_t] == j) {
        continue;
      }
      const auto& task = _input.jobs[tw_r.route[rev_t]];
      if (task.tws.back().end < job_available + job.services[v_type] +
                                  vehicle.duration(job_index, task.index())) {
        // Job is available too late to be performed any time before
        // task at rev_t solely based on its TW.
        weak_insertion_ranks_begin[v][j] = rev_t + 1;
        assert(weak_insertion_ranks_begin[v][j] <= insertion_ranks_begin[v][j]);
        break;
      }
    }
  }
}

void SolutionState::update_cheapest_job_rank_in_routes(
  const std::vector<Index>& route_1,
  const std::vector<Index>& route_2,
  Index v1,
  Index v2) {
  cheapest_job_rank_in_routes_from[v1][v2].assign(route_1.size(), 0);
  cheapest_job_rank_in_routes_to[v1][v2].assign(route_1.size(), 0);

  for (std::size_t r1 = 0; r1 < route_1.size(); ++r1) {
    const Index index_r1 = _input.jobs[route_1[r1]].index();

    auto min_from = std::numeric_limits<Cost>::max();
    auto min_to = std::numeric_limits<Cost>::max();
    Index best_from_rank = 0;
    Index best_to_rank = 0;

    const auto& vehicle = _input.vehicles[v2];
    for (std::size_t r2 = 0; r2 < route_2.size(); ++r2) {
      const Index index_r2 = _input.jobs[route_2[r2]].index();
      if (const auto cost_from = vehicle.cost(index_r1, index_r2);
          cost_from < min_from) {
        min_from = cost_from;
        best_from_rank = r2;
      }
      const auto cost_to = vehicle.cost(index_r2, index_r1);
      if (cost_to < min_to) {
        min_to = cost_to;
        best_to_rank = r2;
      }
    }

    cheapest_job_rank_in_routes_from[v1][v2][r1] = best_from_rank;
    cheapest_job_rank_in_routes_to[v1][v2][r1] = best_to_rank;
  }
}

void SolutionState::update_route_eval(const std::vector<Index>& route,
                                      Index v) {
  route_evals[v] = route_eval_for_vehicle(_input, v, route);
}

void SolutionState::update_route_bbox(const std::vector<Index>& route,
                                      Index v) {
  if (_input.all_locations_have_coords()) {
    auto& bbox = route_bbox[v];
    bbox = BBox();

    std::ranges::for_each(route, [this, &bbox](const auto i) {
      const auto& loc = _input.jobs[i].location;
      assert(loc.has_coordinates());
      bbox.extend(loc.coordinates());
    });
  }
}

template void SolutionState::setup(const std::vector<RawRoute>&);
template void SolutionState::setup(const std::vector<TWRoute>&);

} // namespace vroom::utils
