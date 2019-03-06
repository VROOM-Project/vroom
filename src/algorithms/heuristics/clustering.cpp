/*
This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <unordered_set>

#include "algorithms/heuristics/clustering.h"
#include "structures/vroom/amount.h"

namespace vroom {
namespace heuristics {

Clustering::Clustering(const Input& input, CLUSTERING t, INIT i, float c)
  : _input(input),
    _type(t),
    _init(i),
    _regret_coeff(c),
    clusters(input.vehicles.size()),
    edges_cost(0),
    assigned_jobs(0),
    non_empty_clusters(0) {
  std::string strategy;
  switch (_type) {
  case CLUSTERING::PARALLEL:
    this->parallel_clustering();
    strategy = "parallel";
    break;
  case CLUSTERING::SEQUENTIAL:
    this->sequential_clustering();
    strategy = "sequential";
    break;
  }
  std::string init_str;
  switch (_init) {
  case INIT::NONE:
    init_str = "none";
    break;
  case INIT::HIGHER_AMOUNT:
    init_str = "higher_amount";
    break;
  case INIT::NEAREST:
    init_str = "nearest";
    break;
  case INIT::FURTHEST:
    init_str = "furthest";
    break;
  case INIT::EARLIEST_DEADLINE:
    assert(false);
    break;
  }

  non_empty_clusters = std::count_if(clusters.begin(),
                                     clusters.end(),
                                     [](auto& c) { return !c.empty(); });
}

inline void update_cost(Index from_index,
                        std::vector<Cost>& costs,
                        std::vector<Index>& parents,
                        const std::vector<Index>& candidates,
                        const std::vector<Job>& jobs,
                        const Matrix<Cost>& m) {
  // Update cost of reaching all candidates (seen as neighbours of
  // "from_index").
  for (auto j : candidates) {
    auto current_cost =
      std::min(m[from_index][jobs[j].index()], m[jobs[j].index()][from_index]);
    if (current_cost < costs[j]) {
      costs[j] = current_cost;
      parents[j] = from_index;
    }
  }
}

void Clustering::parallel_clustering() {
  auto V = _input.vehicles.size();
  auto J = _input.jobs.size();
  auto& jobs = _input.jobs;
  auto& vehicles = _input.vehicles;
  auto m = _input.get_matrix();

  // Current best known costs to add jobs to vehicle clusters.
  std::vector<std::vector<Cost>>
    costs(V, std::vector<Cost>(J, std::numeric_limits<Cost>::max()));

  // For each vehicle cluster, we need to maintain a vector of job
  // candidates (represented by their index in 'jobs'). Initialization
  // updates all costs related to start/end for each vehicle cluster.
  std::vector<std::vector<Index>> candidates(V);

  // Remember wanabee parent for each job in each cluster.
  std::vector<std::vector<Index>> parents(V, std::vector<Index>(J));

  for (std::size_t v = 0; v < V; ++v) {
    // Only keep jobs compatible with vehicle skills in candidates.
    for (std::size_t j = 0; j < J; ++j) {
      if (_input.vehicle_ok_with_job(v, j) and
          jobs[j].amount <= _input.vehicles[v].capacity) {
        candidates[v].push_back(j);
      }
    }

    if (vehicles[v].has_start()) {
      auto start_index = vehicles[v].start.get().index();
      update_cost(start_index, costs[v], parents[v], candidates[v], jobs, m);

      if (vehicles[v].has_end()) {
        auto end_index = vehicles[v].end.get().index();
        if (start_index != end_index) {
          update_cost(end_index, costs[v], parents[v], candidates[v], jobs, m);
        }
      }
    } else {
      assert(vehicles[v].has_end());
      auto end_index = vehicles[v].end.get().index();
      update_cost(end_index, costs[v], parents[v], candidates[v], jobs, m);
    }
  }

  // Remember current capacity left in clusters.
  std::vector<Amount> capacities;
  for (std::size_t v = 0; v < V; ++v) {
    capacities.emplace_back(vehicles[v].capacity);
  }

  // Regrets[v][j] is the min cost of reaching jobs[j] from another
  // cluster than v. It serves as an indicator of the cost we'll have
  // to support later when NOT including a job to the current cluster.
  std::vector<std::vector<Cost>> regrets(V, std::vector<Cost>(J, 0));
  for (std::size_t v = 0; v < V; ++v) {
    for (auto j : candidates[v]) {
      auto current_regret = std::numeric_limits<Cost>::max();
      for (std::size_t other_v = 0; other_v < V; ++other_v) {
        // Cost from other clusters that potentially can handle job.
        if ((v == other_v) or
            (costs[other_v][j] == std::numeric_limits<Cost>::max())) {
          continue;
        }
        current_regret = std::min(current_regret, costs[other_v][j]);
      }
      regrets[v][j] = current_regret;
    }
  }

  // Cluster initialization: define available initialization
  // strategies then run initialization sequentially on all clusters.

  // Initialize cluster with the job that has "higher" amount (and is
  // the further away in case of amount tie).
  auto higher_amount_init_lambda = [&](auto v) {
    return [&, v](Index lhs, Index rhs) {
      return jobs[lhs].amount << jobs[rhs].amount or
             (jobs[lhs].amount == jobs[rhs].amount and
              costs[v][lhs] < costs[v][rhs]);
    };
  };
  // Initialize cluster with the nearest job.
  auto nearest_init_lambda = [&](auto v) {
    return
      [&, v](Index lhs, Index rhs) { return costs[v][lhs] < costs[v][rhs]; };
  };

  if (_init != INIT::NONE) {
    for (std::size_t v = 0; v < V; ++v) {
      auto init_job = candidates[v].cend();
      if (_init == INIT::HIGHER_AMOUNT) {
        init_job = std::max_element(candidates[v].cbegin(),
                                    candidates[v].cend(),
                                    higher_amount_init_lambda(v));
      }
      if (_init == INIT::NEAREST) {
        init_job = std::min_element(candidates[v].cbegin(),
                                    candidates[v].cend(),
                                    nearest_init_lambda(v));
      }

      if (_init == INIT::FURTHEST) {
        init_job = std::max_element(candidates[v].cbegin(),
                                    candidates[v].cend(),
                                    nearest_init_lambda(v));
      }

      if (init_job != candidates[v].cend()) {
        auto job_rank = *init_job;
        clusters[v].push_back(job_rank);
        ++assigned_jobs;
        edges_cost += costs[v][job_rank];
        capacities[v] -= jobs[job_rank].amount;
        candidates[v].erase(init_job);

        update_cost(jobs[job_rank].index(),
                    costs[v],
                    parents[v],
                    candidates[v],
                    jobs,
                    m);
        // Update regrets as costs from matching cluster to job
        // candidates potentially decreases.
        for (auto j : candidates[v]) {
          auto new_cost = std::min(m[jobs[job_rank].index()][jobs[j].index()],
                                   m[jobs[j].index()][jobs[job_rank].index()]);
          for (std::size_t other_v = 0; other_v < V; ++other_v) {
            // Regret for other clusters that potentially can handle job.
            if ((other_v == v) or
                (costs[other_v][j] == std::numeric_limits<Cost>::max())) {
              continue;
            }
            regrets[other_v][j] = std::min(regrets[other_v][j], new_cost);
          }
        }

        for (std::size_t other_v = 0; other_v < V; ++other_v) {
          if (other_v != v) {
            auto search = std::find(candidates[other_v].begin(),
                                    candidates[other_v].end(),
                                    job_rank);
            if (search != candidates[other_v].end()) {
              candidates[other_v].erase(search);
            }
          }
        }
      }
    }
  }

  auto eval_lambda = [&](auto v) {
    return [&, v](auto i, auto j) {
      return _regret_coeff * static_cast<float>(regrets[v][i]) -
               static_cast<float>(costs[v][i]) <
             _regret_coeff * static_cast<float>(regrets[v][j]) -
               static_cast<float>(costs[v][j]);
    };
  };

  bool candidates_remaining = true;

  while (candidates_remaining) {
    // Remember best cluster and job candidate.
    bool capacity_ok = false;
    Index best_v = 0; // Dummy init, value never used.
    Index best_j;
    Cost best_cost = std::numeric_limits<Cost>::max();

    for (std::size_t v = 0; v < V; ++v) {
      if (candidates[v].empty()) {
        continue;
      }

      // Consider best job candidate for current cluster.
      std::nth_element(candidates[v].begin(),
                       candidates[v].end() - 1,
                       candidates[v].end(),
                       eval_lambda(v));

      auto current_j = candidates[v].back();
      if (jobs[current_j].amount <= capacities[v] and
          (costs[v][current_j] < best_cost or
           (costs[v][current_j] == best_cost and capacities[best_v]
                                                   << capacities[v]))) {
        // Update if job candidate is OK wrt capacity and cheaper to
        // add. In case of cost tie, pick cluster with "higher"
        // capacity.
        capacity_ok = true;
        best_v = v;
        best_j = current_j;
        best_cost = costs[v][best_j];
      }
    }

    // Stopping condition changed below in two cases. First situation:
    // no doable job was added due to capacity constraints, so drop
    // all best candidates and explore further. Second situation: some
    // cluster gets a job addition and other candidates remain.
    candidates_remaining = false;

    if (!capacity_ok) {
      // Removing all cheapest candidates as none is doable with
      // regard to capacity.
      for (std::size_t v = 0; v < V; ++v) {
        if (candidates[v].empty()) {
          continue;
        }
        candidates[v].pop_back();

        candidates_remaining |= !candidates[v].empty();
      }
      continue;
    }

    // Add best candidate to matching cluster and remove from all
    // candidate vectors.
    clusters[best_v].push_back(best_j);
    ++assigned_jobs;
    edges_cost += best_cost;
    capacities[best_v] -= jobs[best_j].amount;

    candidates[best_v].pop_back();
    update_cost(jobs[best_j].index(),
                costs[best_v],
                parents[best_v],
                candidates[best_v],
                jobs,
                m);
    // Update regrets as costs from matching cluster to job candidates
    // potentially decreases.
    for (auto j : candidates[best_v]) {
      auto new_cost = std::min(m[jobs[best_j].index()][jobs[j].index()],
                               m[jobs[j].index()][jobs[best_j].index()]);
      for (std::size_t other_v = 0; other_v < V; ++other_v) {
        // Regret for other clusters that potentially can handle job.
        if ((other_v == best_v) or
            (costs[other_v][j] == std::numeric_limits<Cost>::max())) {
          continue;
        }
        regrets[other_v][j] = std::min(regrets[other_v][j], new_cost);
      }
    }

    for (std::size_t v = 0; v < V; ++v) {
      if (v != best_v) {
        auto search =
          std::find(candidates[v].begin(), candidates[v].end(), best_j);
        if (search != candidates[v].end()) {
          candidates[v].erase(search);
        }
      }

      candidates_remaining |= !candidates[v].empty();
    }
  }
}

void Clustering::sequential_clustering() {
  auto V = _input.vehicles.size();
  auto J = _input.jobs.size();
  auto& jobs = _input.jobs;
  auto& vehicles = _input.vehicles;
  auto m = _input.get_matrix();

  // For each vehicle cluster, we need to initialize a vector of job
  // candidates (represented by their index in 'jobs').
  std::unordered_set<Index> candidates_set;
  for (Index i = 0; i < J; ++i) {
    candidates_set.insert(i);
  }

  // Remember initial cost of reaching a job from a vehicle (based on
  // start/end loc).
  std::vector<std::vector<Cost>> vehicles_to_job_costs(V, std::vector<Cost>(J));

  for (std::size_t j = 0; j < J; ++j) {
    for (std::size_t v = 0; v < V; ++v) {
      Cost current_cost = std::numeric_limits<Cost>::max();
      if (vehicles[v].has_start()) {
        auto start_index = vehicles[v].start.get().index();
        current_cost = std::min(current_cost, m[start_index][jobs[j].index()]);
      }
      if (vehicles[v].has_end()) {
        auto end_index = vehicles[v].end.get().index();
        current_cost = std::min(current_cost, m[jobs[j].index()][end_index]);
      }
      vehicles_to_job_costs[v][j] = current_cost;
    }
  }

  // Regrets[v][j] is the min cost of reaching jobs[j] from another
  // yet-to-build cluster after v. It serves as an indicator of the
  // cost we'll have to support later when NOT including a job to the
  // current cluster.
  std::vector<std::vector<Cost>> regrets(V, std::vector<Cost>(J, 0));

  if (vehicles.size() > 1) {
    // Regret for penultimate cluster is the cost for last
    // vehicle. Previous values are computed backward.
    for (std::size_t j = 0; j < J; ++j) {
      regrets[V - 2][j] = vehicles_to_job_costs[V - 1][j];
    }
    for (std::size_t i = 3; i <= V; ++i) {
      for (std::size_t j = 0; j < J; ++j) {
        regrets[V - i][j] =
          std::min(regrets[V - i + 1][j], vehicles_to_job_costs[V - i + 1][j]);
      }
    }
  }

  // Define available initialization strategies.

  // Initialize cluster with the job that has "higher" amount (and is
  // the further away in case of amount tie).
  auto higher_amount_init_lambda = [&](auto v) {
    return [&, v](Index lhs, Index rhs) {
      return jobs[lhs].amount << jobs[rhs].amount or
             (jobs[lhs].amount == jobs[rhs].amount and
              vehicles_to_job_costs[v][lhs] < vehicles_to_job_costs[v][rhs]);
    };
  };
  // Initialize cluster with the nearest job.
  auto nearest_init_lambda = [&](auto v) {
    return [&, v](Index lhs, Index rhs) {
      return vehicles_to_job_costs[v][lhs] < vehicles_to_job_costs[v][rhs];
    };
  };

  for (std::size_t v = 0; v < V; ++v) {
    // Initialization with remaining compatible jobs while remembering
    // costs to jobs for current vehicle.
    std::vector<Index> candidates;
    for (auto i : candidates_set) {
      if (_input.vehicle_ok_with_job(v, i) and
          jobs[i].amount <= _input.vehicles[v].capacity) {
        candidates.push_back(i);
      }
    }

    // Current best known costs to add jobs to current vehicle cluster.
    std::vector<Cost> costs(J, std::numeric_limits<Cost>::max());

    // Remember wanabee parent for each job.
    std::vector<Index> parents(J);

    // Updating costs related to start/end for each vehicle cluster.
    if (vehicles[v].has_start()) {
      auto start_index = vehicles[v].start.get().index();
      update_cost(start_index, costs, parents, candidates, jobs, m);

      if (vehicles[v].has_end()) {
        auto end_index = vehicles[v].end.get().index();
        if (start_index != end_index) {
          update_cost(end_index, costs, parents, candidates, jobs, m);
        }
      }
    } else {
      assert(vehicles[v].has_end());
      auto end_index = vehicles[v].end.get().index();
      update_cost(end_index, costs, parents, candidates, jobs, m);
    }

    // Remember current capacity left in cluster.
    auto capacity = vehicles[v].capacity;

    // Strategy for cluster initialization.
    if (_init != INIT::NONE) {
      auto init_job = candidates.cend();
      if (_init == INIT::HIGHER_AMOUNT) {
        init_job = std::max_element(candidates.cbegin(),
                                    candidates.cend(),
                                    higher_amount_init_lambda(v));
      }
      if (_init == INIT::NEAREST) {
        init_job = std::min_element(candidates.cbegin(),
                                    candidates.cend(),
                                    nearest_init_lambda(v));
      }
      if (_init == INIT::FURTHEST) {
        init_job = std::max_element(candidates.cbegin(),
                                    candidates.cend(),
                                    nearest_init_lambda(v));
      }

      if (init_job != candidates.cend()) {
        auto job_rank = *init_job;
        clusters[v].push_back(job_rank);
        ++assigned_jobs;
        edges_cost += vehicles_to_job_costs[v][job_rank];
        capacity -= jobs[job_rank].amount;
        candidates_set.erase(job_rank);
        candidates.erase(init_job);

        update_cost(jobs[job_rank].index(),
                    costs,
                    parents,
                    candidates,
                    jobs,
                    m);
      }
    }

    auto eval_lambda = [&](auto i, auto j) {
      return _regret_coeff * static_cast<float>(regrets[v][i]) -
               static_cast<float>(costs[i]) <
             _regret_coeff * static_cast<float>(regrets[v][j]) -
               static_cast<float>(costs[j]);
    };

    while (!candidates.empty()) {
      std::nth_element(candidates.begin(),
                       candidates.end() - 1,
                       candidates.end(),
                       eval_lambda);

      auto current_j = candidates.back();

      if (jobs[current_j].amount <= capacity) {
        clusters[v].push_back(current_j);
        ++assigned_jobs;
        edges_cost += costs[current_j];
        capacity -= jobs[current_j].amount;
        candidates_set.erase(current_j);

        update_cost(jobs[current_j].index(),
                    costs,
                    parents,
                    candidates,
                    jobs,
                    m);
      }

      candidates.pop_back();
    }
  }
}

} // namespace heuristics
} // namespace vroom
