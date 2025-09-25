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
    fwd_evals(_nb_vehicles, std::vector<std::vector<Eval>>(_nb_vehicles)),
    bwd_evals(_nb_vehicles, std::vector<std::vector<Eval>>(_nb_vehicles)),
    service_evals(_nb_vehicles, std::vector<std::vector<Eval>>(_nb_vehicles)),
    fwd_setup_evals(_nb_vehicles, std::vector<std::vector<Eval>>(_nb_vehicles)),
    bwd_setup_evals(_nb_vehicles, std::vector<std::vector<Eval>>(_nb_vehicles)),
    fwd_skill_rank(_nb_vehicles, std::vector<Index>(_nb_vehicles)),
    bwd_skill_rank(_nb_vehicles, std::vector<Index>(_nb_vehicles)),
    fwd_priority(_nb_vehicles),
    bwd_priority(_nb_vehicles),
    edge_evals_around_node(_nb_vehicles),
    node_gains(_nb_vehicles),
    edge_evals_around_edge(_nb_vehicles),
    edge_gains(_nb_vehicles),
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

void SolutionState::setup(const RawRoute& r) {
  update_costs(r);
  update_skills(r);
  update_priorities(r);
  set_node_gains(r);
  set_edge_gains(r);
  set_pd_matching_ranks(r);
  set_pd_gains(r);
  set_insertion_ranks(r);
  update_route_eval(r);
  update_route_bbox(r);
}

template <class Route>
void SolutionState::setup(const std::vector<Route>& sol) {
  for (std::size_t v = 0; v < _nb_vehicles; ++v) {
    setup(sol[v]);
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

void SolutionState::update_costs(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;
  const auto& route = raw_route.route;

  fwd_evals[v] =
    std::vector<std::vector<Eval>>(_nb_vehicles,
                                   std::vector<Eval>(route.size()));
  bwd_evals[v] =
    std::vector<std::vector<Eval>>(_nb_vehicles,
                                   std::vector<Eval>(route.size()));

  fwd_setup_evals[v] =
    std::vector<std::vector<Eval>>(_nb_vehicles,
                                   std::vector<Eval>(route.size()));
  bwd_setup_evals[v] =
    std::vector<std::vector<Eval>>(_nb_vehicles,
                                   std::vector<Eval>(route.size()));

  service_evals[v] =
    std::vector<std::vector<Eval>>(_nb_vehicles,
                                   std::vector<Eval>(route.size()));

  if (route.empty()) {
    return;
  }

  // Handle evals for first job.
  const auto& first_job = _input.jobs[route[0]];
  const auto first_index = first_job.index();
  const auto& last_job = _input.jobs[route.back()];
  const auto last_index = last_job.index();

  for (Index v_rank = 0; v_rank < _nb_vehicles; ++v_rank) {
    fwd_evals[v][v_rank][0] = Eval();
    bwd_evals[v][v_rank][0] = Eval();

    const auto& vehicle = _input.vehicles[v_rank];
    const auto service_eval =
      vehicle.task_eval(first_job.services[vehicle.type]);

    service_evals[v][v_rank][0] = service_eval;

    if (!vehicle.has_start() || vehicle.start.value().index() != first_index) {
      fwd_setup_evals[v][v_rank][0] =
        vehicle.task_eval(first_job.setups[vehicle.type]);
    }

    if (!vehicle.has_start() || vehicle.start.value().index() != last_index) {
      bwd_setup_evals[v][v_rank].back() =
        vehicle.task_eval(last_job.setups[vehicle.type]);
    }
  }

  for (std::size_t i = 1; i < route.size(); ++i) {
    const auto& previous_job = _input.jobs[route[i - 1]];
    const auto& current_job = _input.jobs[route[i]];

    const auto previous_index = previous_job.index();
    const auto current_index = current_job.index();
    const bool apply_setup = (previous_index != current_index);

    for (Index v_rank = 0; v_rank < _nb_vehicles; ++v_rank) {
      const auto& vehicle = _input.vehicles[v_rank];
      fwd_evals[v][v_rank][i] = fwd_evals[v][v_rank][i - 1] +
                                vehicle.eval(previous_index, current_index);

      bwd_evals[v][v_rank][i] = bwd_evals[v][v_rank][i - 1] +
                                vehicle.eval(current_index, previous_index);

      const auto service_eval =
        vehicle.task_eval(current_job.services[vehicle.type]);
      service_evals[v][v_rank][i] =
        service_evals[v][v_rank][i - 1] + service_eval;

      fwd_setup_evals[v][v_rank][i] = fwd_setup_evals[v][v_rank][i - 1];
      if (apply_setup) {
        fwd_setup_evals[v][v_rank][i] +=
          vehicle.task_eval(current_job.setups[vehicle.type]);
      }
    }
  }

  // Handling bwd_setup_evals only.
  for (std::size_t i = route.size() - 1; i > 0; --i) {
    const auto& previous_job = _input.jobs[route[i]];
    const auto& current_job = _input.jobs[route[i - 1]];
    const bool apply_setup = (previous_job.index() != current_job.index());

    for (Index v_rank = 0; v_rank < _nb_vehicles; ++v_rank) {
      bwd_setup_evals[v][v_rank][i - 1] = bwd_setup_evals[v][v_rank][i];
      if (apply_setup) {
        const auto& vehicle = _input.vehicles[v_rank];
        bwd_setup_evals[v][v_rank][i - 1] +=
          vehicle.task_eval(current_job.setups[vehicle.type]);
      }
    }
  }
}

void SolutionState::update_skills(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;
  const auto& route = raw_route.route;

  for (std::size_t v2 = 0; v2 < _nb_vehicles; ++v2) {
    if (v == v2) {
      continue;
    }

    auto fwd = std::ranges::find_if_not(route, [&](auto j_rank) {
      return _input.vehicle_ok_with_job(v2, j_rank);
    });
    fwd_skill_rank[v][v2] = std::distance(route.begin(), fwd);

    auto bwd = std::find_if_not(route.rbegin(), route.rend(), [&](auto j_rank) {
      return _input.vehicle_ok_with_job(v2, j_rank);
    });
    bwd_skill_rank[v][v2] = route.size() - std::distance(route.rbegin(), bwd);
  }
}

void SolutionState::update_priorities(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;
  const auto& route = raw_route.route;

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

void SolutionState::set_node_gains(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;
  const auto& route = raw_route.route;

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

  Duration task_duration_gain = _input.jobs[route[0]].setups[vehicle.type] +
                                _input.jobs[route[0]].services[vehicle.type];

  if (vehicle.has_start()) {
    // There is a previous step before job at rank 0.
    p_index = vehicle.start.value().index();
    previous_eval = vehicle.eval(p_index, c_index);

    if (p_index == c_index) {
      task_duration_gain -= _input.jobs[route[0]].setups[vehicle.type];
    }
    // Update next_eval with next job or end.
    if (route.size() > 1) {
      n_index = _input.jobs[route[1]].index();
      next_eval = vehicle.eval(c_index, n_index);
      new_edge_eval = vehicle.eval(p_index, n_index);

      if (n_index == c_index && p_index != n_index) {
        task_duration_gain -= _input.jobs[route[1]].setups[vehicle.type];
      }
      if (n_index != c_index && p_index == n_index) {
        task_duration_gain += _input.jobs[route[1]].setups[vehicle.type];
      }
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

      if (n_index == c_index) {
        task_duration_gain -= _input.jobs[route[1]].setups[vehicle.type];
      }
    } else {
      assert(vehicle.has_end());
      n_index = vehicle.end.value().index();
    }
    next_eval = vehicle.eval(c_index, n_index);
  }

  edge_evals_around_node[v][0] = previous_eval + next_eval;

  node_gains[v][0] = edge_evals_around_node[v][0] - new_edge_eval +
                     vehicle.task_eval(task_duration_gain);

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

    task_duration_gain = _input.jobs[route[i]].services[vehicle.type];

    if (p_index != c_index) {
      task_duration_gain += _input.jobs[route[i]].setups[vehicle.type];
    }

    if (n_index == c_index && p_index != n_index) {
      task_duration_gain -= _input.jobs[route[i + 1]].setups[vehicle.type];
    }
    if (n_index != c_index && p_index == n_index) {
      task_duration_gain += _input.jobs[route[i + 1]].setups[vehicle.type];
    }

    edge_evals_around_node[v][i] =
      vehicle.eval(p_index, c_index) + vehicle.eval(c_index, n_index);

    node_gains[v][i] = edge_evals_around_node[v][i] -
                       vehicle.eval(p_index, n_index) +
                       vehicle.task_eval(task_duration_gain);
  }

  // Handling last job after a previous job is special due to
  // potential open tours.
  auto last_rank = route.size() - 1;
  c_index = _input.jobs[route[last_rank]].index();

  assert(route.size() > 1);
  p_index = _input.jobs[route[last_rank - 1]].index();
  previous_eval = vehicle.eval(p_index, c_index);

  task_duration_gain = _input.jobs[route[last_rank]].services[vehicle.type];
  if (p_index != c_index) {
    task_duration_gain += _input.jobs[route[last_rank]].setups[vehicle.type];
  }

  next_eval = Eval();
  new_edge_eval = Eval();

  if (vehicle.has_end()) {
    // There is a next step after last job.
    n_index = vehicle.end.value().index();
    next_eval = vehicle.eval(c_index, n_index);
    new_edge_eval = vehicle.eval(p_index, n_index);
  }

  edge_evals_around_node[v][last_rank] = previous_eval + next_eval;

  node_gains[v][last_rank] = edge_evals_around_node[v][last_rank] -
                             new_edge_eval +
                             vehicle.task_eval(task_duration_gain);
}

void SolutionState::set_edge_gains(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;
  const auto& route = raw_route.route;

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

  edge_evals_around_edge[v][0] = previous_eval + next_eval;

  edge_gains[v][0] = edge_evals_around_edge[v][0] - new_edge_eval;

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

    edge_evals_around_edge[v][i] =
      vehicle.eval(p_index, c_index) + vehicle.eval(after_c_index, n_index);

    edge_gains[v][i] =
      edge_evals_around_edge[v][i] - vehicle.eval(p_index, n_index);
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

  edge_evals_around_edge[v][last_edge_rank] = previous_eval + next_eval;

  edge_gains[v][last_edge_rank] =
    edge_evals_around_edge[v][last_edge_rank] - new_edge_eval;
}

void SolutionState::set_pd_gains(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;
  const auto& route = raw_route.route;

  pd_gains[v] = std::vector<Eval>(route.size());

  for (std::size_t pickup_rank = 0; pickup_rank < route.size(); ++pickup_rank) {
    if (_input.jobs[route[pickup_rank]].type != JOB_TYPE::PICKUP) {
      continue;
    }
    const Index delivery_rank = matching_delivery_rank[v][pickup_rank];

    if (pickup_rank + 1 == delivery_rank) {
      // Pickup and delivery in a row.
      pd_gains[v][pickup_rank] = utils::removal_gain(_input,
                                                     *this,
                                                     raw_route,
                                                     pickup_rank,
                                                     pickup_rank + 2);
    } else {
      // Simply add both gains as neighbouring edges are disjoint.
      pd_gains[v][pickup_rank] =
        node_gains[v][pickup_rank] + node_gains[v][delivery_rank];
    }
  }
}

void SolutionState::set_pd_matching_ranks(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;
  const auto& route = raw_route.route;

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

void SolutionState::set_insertion_ranks(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;
  const auto& route = raw_route.route;

  insertion_ranks_end[v] =
    std::vector<Index>(_input.jobs.size(), route.size() + 1);
  insertion_ranks_begin[v] = std::vector<Index>(_input.jobs.size(), 0);

  weak_insertion_ranks_end[v] =
    std::vector<Index>(_input.jobs.size(), route.size() + 1);
  weak_insertion_ranks_begin[v] = std::vector<Index>(_input.jobs.size(), 0);
}

void SolutionState::set_insertion_ranks(const TWRoute& tw_r) {
  const auto v = tw_r.v_rank;
  const auto& route = tw_r.route;

  insertion_ranks_end[v] =
    std::vector<Index>(_input.jobs.size(), route.size() + 1);
  insertion_ranks_begin[v] = std::vector<Index>(_input.jobs.size(), 0);

  weak_insertion_ranks_end[v] =
    std::vector<Index>(_input.jobs.size(), route.size() + 1);
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
    for (std::size_t t = 0; t < route.size(); ++t) {
      if (route[t] == j) {
        continue;
      }
      if (job_deadline <
          tw_r.earliest[t] + tw_r.action_time[t] +
            vehicle.duration(_input.jobs[route[t]].index(), job_index)) {
        // Too late to perform job any time after task at t based on
        // its earliest date in route for v.
        insertion_ranks_end[v][j] = t + 1;
        break;
      }
    }
    for (std::size_t t = 0; t < route.size(); ++t) {
      const auto rev_t = route.size() - 1 - t;
      if (route[rev_t] == j) {
        continue;
      }
      if (tw_r.latest[rev_t] <
          job_available + job.services[v_type] +
            vehicle.duration(job_index, _input.jobs[route[rev_t]].index())) {
        // Job is available too late to be performed any time before
        // task at rev_t based on its latest date in route for v.
        insertion_ranks_begin[v][j] = rev_t + 1;
        break;
      }
    }

    // Handle weak_insertion_ranks_*
    for (std::size_t t = 0; t < route.size(); ++t) {
      if (route[t] == j) {
        continue;
      }
      const auto& task = _input.jobs[route[t]];
      if (job_deadline < task.tws.front().start + task.services[v_type] +
                           vehicle.duration(task.index(), job_index)) {
        // Too late to perform job any time after task at t solely
        // based on its TW.
        weak_insertion_ranks_end[v][j] = t + 1;
        assert(insertion_ranks_end[v][j] <= weak_insertion_ranks_end[v][j]);
        break;
      }
    }
    for (std::size_t t = 0; t < route.size(); ++t) {
      const auto rev_t = route.size() - 1 - t;
      if (route[rev_t] == j) {
        continue;
      }
      const auto& task = _input.jobs[route[rev_t]];
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

void SolutionState::update_route_eval(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;

  route_evals[v] = route_eval_for_vehicle(_input, v, raw_route.route);
}

void SolutionState::update_route_bbox(const RawRoute& raw_route) {
  const auto v = raw_route.v_rank;

  if (_input.all_locations_have_coords()) {
    auto& bbox = route_bbox[v];
    bbox = BBox();

    std::ranges::for_each(raw_route.route, [this, &bbox](const auto i) {
      const auto& loc = _input.jobs[i].location;
      assert(loc.has_coordinates());
      bbox.extend(loc.coordinates());
    });
  }
}

template void SolutionState::setup(const std::vector<RawRoute>&);
template void SolutionState::setup(const std::vector<TWRoute>&);

} // namespace vroom::utils
