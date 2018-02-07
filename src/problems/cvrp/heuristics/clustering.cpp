/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "clustering.h"

inline void update_cost(index_t from_index,
                        std::vector<cost_t>& costs,
                        std::vector<index_t>& parents,
                        const std::vector<index_t>& candidates,
                        const std::vector<job_t>& jobs,
                        const matrix<cost_t>& m) {
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

std::vector<std::vector<index_t>> clustering(const input& input) {
  auto V = input._vehicles.size();
  auto J = input._jobs.size();
  auto& jobs = input._jobs;
  auto& vehicles = input._vehicles;
  auto& m = input._matrix;

  // Vehicle clusters.
  std::vector<std::vector<index_t>> clusters(V);

  // Current best known costs to add jobs to vehicle clusters.
  std::vector<std::vector<cost_t>>
    costs(V, std::vector<cost_t>(J, std::numeric_limits<cost_t>::max()));

  // For each vehicle cluster, we need to maintain a vector of job
  // candidates (represented by their index in 'jobs'). Initialization
  // also include pushing start/end into vehicle clusters and updating
  // costs accordingly.
  std::vector<std::vector<index_t>> candidates(V, std::vector<index_t>(J));

  // Remember wanabee parent for each job in each cluster.
  std::vector<std::vector<index_t>> parents(V, std::vector<index_t>(J));

  for (std::size_t v = 0; v < V; ++v) {
    // TODO, only keep jobs compatible with vehicle skills.
    std::iota(candidates[v].begin(), candidates[v].end(), 0);

    if (vehicles[v].has_start()) {
      auto start_index = vehicles[v].start.get().index();
      clusters[v].push_back(start_index);
      update_cost(start_index, costs[v], parents[v], candidates[v], jobs, m);

      if (vehicles[v].has_end()) {
        auto end_index = vehicles[v].end.get().index();
        if (start_index != end_index) {
          clusters[v].push_back(end_index);
          update_cost(end_index, costs[v], parents[v], candidates[v], jobs, m);
        }
      }
    } else {
      assert(vehicles[v].has_end());
      auto end_index = vehicles[v].end.get().index();
      clusters[v].push_back(end_index);
      update_cost(end_index, costs[v], parents[v], candidates[v], jobs, m);
    }
  }

  // Remember current capacity left in clusters.
  std::vector<amount_t> capacities;
  for (std::size_t v = 0; v < V; ++v) {
    capacities.emplace_back(vehicles[v].capacity.get());
  }

  bool candidates_remaining = true;

  while (candidates_remaining) {
    // Remember best cluster and job candidate.
    bool capacity_ok = false;
    index_t best_v = 0; // Dummy init, value never used.
    index_t best_j;
    cost_t best_cost = std::numeric_limits<cost_t>::max();

    for (std::size_t v = 0; v < V; ++v) {
      if (candidates[v].empty()) {
        continue;
      }

      // Consider best job candidate for current cluster.
      std::make_heap(candidates[v].begin(),
                     candidates[v].end(),
                     [&](auto i, auto j) { return costs[v][i] > costs[v][j]; });

      auto current_j = candidates[v].front();
      if (jobs[current_j].amount.get() <= capacities[v] and
          (costs[v][current_j] < best_cost or
           (costs[v][current_j] == best_cost and
            capacities[best_v] < capacities[v]))) {
        // Update if job candidate is OK wrt capacity and cheaper to
        // add. In case of cost tie, pick cluster with most remaining
        // room.
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
        std::pop_heap(candidates[v].begin(),
                      candidates[v].end(),
                      [&](auto i, auto j) {
                        return costs[v][i] > costs[v][j];
                      });
        candidates[v].pop_back();

        candidates_remaining |= !candidates[v].empty();
      }
      continue;
    }

    // Add best candidate to matching cluster and remove from all
    // candidate vectors.
    clusters[best_v].push_back(jobs[best_j].index());
    std::cout << vehicles[best_v].id << ";" << parents[best_v][best_j] << "->"
              << jobs[best_j].index() << std::endl;
    capacities[best_v] -= jobs[best_j].amount.get();

    std::pop_heap(candidates[best_v].begin(),
                  candidates[best_v].end(),
                  [&](auto i, auto j) {
                    return costs[best_v][i] > costs[best_v][j];
                  });
    candidates[best_v].pop_back();
    update_cost(jobs[best_j].index(),
                costs[best_v],
                parents[best_v],
                candidates[best_v],
                jobs,
                m);

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

  return clusters;
}

std::vector<std::vector<index_t>> sequential_clustering(const input& input,
                                                        double regret_coeff) {
  auto V = input._vehicles.size();
  auto J = input._jobs.size();
  auto& jobs = input._jobs;
  auto& vehicles = input._vehicles;
  auto& m = input._matrix;

  // Vehicle clusters.
  std::vector<std::vector<index_t>> clusters(V);

  // For each vehicle cluster, we need to initialize a vector of job
  // candidates (represented by their index in 'jobs').
  std::unordered_set<index_t> candidates_set;
  for (index_t i = 0; i < J; ++i) {
    candidates_set.insert(i);
  }

  // Remember initial cost of reaching a job from a vehicle (based on
  // start/end loc).
  std::vector<std::vector<cost_t>>
    vehicles_to_job_costs(V, std::vector<cost_t>(J));

  for (std::size_t j = 0; j < J; ++j) {
    for (std::size_t v = 0; v < V; ++v) {
      cost_t current_cost = std::numeric_limits<cost_t>::max();
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
  std::vector<std::vector<cost_t>> regrets(V, std::vector<cost_t>(J, 0));

  // Regret for penultimate cluster is the cost for last
  // vehicle. Previous values are computed backward.
  for (std::size_t j = 0; j < J; ++j) {
    regrets[V - 2][j] = vehicles_to_job_costs[V - 1][j];
  }
  for (std::size_t i = 3; i <= V; ++i) {
    for (std::size_t j = 0; j < J; ++j) {
      regrets[V - i][j] = std::min(regrets[V - i + 1][j],
                                   vehicles_to_job_costs[V - i + 1][j]);
    }
  }

  for (std::size_t v = 0; v < V; ++v) {
    // Initialization with remaining compatible jobs while remembering
    // costs to jobs for current vehicle.
    std::vector<index_t> candidates;
    for (auto i : candidates_set) {
      // TODO, only keep jobs compatible with vehicle skills.
      if (jobs[i].amount.get() <= input._vehicles[v].capacity.get()) {
        candidates.push_back(i);
      }
    }

    // Current best known costs to add jobs to current vehicle cluster.
    std::vector<cost_t> costs(J, std::numeric_limits<cost_t>::max());

    // Remember wanabee parent for each job.
    std::vector<index_t> parents(J);

    // Pushing start/end into vehicle clusters and updating costs
    // accordingly.
    if (vehicles[v].has_start()) {
      auto start_index = vehicles[v].start.get().index();
      clusters[v].push_back(start_index);
      update_cost(start_index, costs, parents, candidates, jobs, m);

      if (vehicles[v].has_end()) {
        auto end_index = vehicles[v].end.get().index();
        if (start_index != end_index) {
          clusters[v].push_back(end_index);
          update_cost(end_index, costs, parents, candidates, jobs, m);
        }
      }
    } else {
      assert(vehicles[v].has_end());
      auto end_index = vehicles[v].end.get().index();
      clusters[v].push_back(end_index);
      update_cost(end_index, costs, parents, candidates, jobs, m);
    }

    // Remember current capacity left in cluster.
    auto capacity = vehicles[v].capacity.get();

    // Initialize cluster with the job that has higher amount (and is
    // the further away in case of amount tie).
    auto init_job
      = std::max_element(candidates.cbegin(),
                         candidates.cend(),
                         [&] (index_t lhs, index_t rhs) {
                           return jobs[lhs].amount.get() < jobs[rhs].amount.get() or
                           (jobs[lhs].amount.get() == jobs[rhs].amount.get() and
                            vehicles_to_job_costs[v][lhs] < vehicles_to_job_costs[v][rhs]);
                         });

    if (init_job != candidates.cend()) {
      auto job_rank = *init_job;
      clusters[v].push_back(jobs[job_rank].index());
      capacity -= jobs[job_rank].amount.get();
      candidates_set.erase(job_rank);
      candidates.erase(init_job);

      std::cout << vehicles[v].id << ";" << parents[job_rank]
                << "->" << jobs[job_rank].index()
                << std::endl;

      update_cost(jobs[job_rank].index(), costs, parents, candidates, jobs, m);
    }

    auto eval_lambda = [&](auto i, auto j) {
      return regret_coeff * static_cast<double>(regrets[v][i]) -
               static_cast<double>(costs[i]) <
             regret_coeff * static_cast<double>(regrets[v][j]) -
               static_cast<double>(costs[j]);
    };

    while (!candidates.empty()) {
      std::make_heap(candidates.begin(), candidates.end(), eval_lambda);

      auto current_j = candidates.front();

      if (jobs[current_j].amount.get() <= capacity) {
        clusters[v].push_back(jobs[current_j].index());
        std::cout << vehicles[v].id << ";" << parents[current_j]
                  << "->" << jobs[current_j].index()
                  << std::endl;
        capacity -= jobs[current_j].amount.get();
        candidates_set.erase(current_j);

        update_cost(jobs[current_j].index(), costs, parents, candidates, jobs, m);
      }

      std::pop_heap(candidates.begin(), candidates.end(), eval_lambda);
      candidates.pop_back();
    }
  }

  return clusters;
}
