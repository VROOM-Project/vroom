/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <thread>

#include <boost/log/trivial.hpp>

#include "problems/cvrp/cvrp.h"
#include "problems/cvrp/heuristics/clustering.h"
#include "problems/cvrp/local_search/local_search.h"
#include "problems/tsp/tsp.h"
#include "structures/vroom/input/input.h"

cvrp::cvrp(const input& input) : vrp(input) {
}

raw_solution cvrp::solve(unsigned nb_threads) const {
  auto nb_tsp = _input._vehicles.size();

  if (nb_tsp == 1 and !_input.has_skills() and _input.amount_size() == 0) {
    // This is a plain TSP, no need to go through the trouble below.
    std::vector<index_t> job_ranks(_input._jobs.size());
    std::iota(job_ranks.begin(), job_ranks.end(), 0);

    tsp p(_input, job_ranks, 0);

    return p.solve(nb_threads);
  }

  struct param {
    CLUSTERING_T type;
    INIT_T init;
    double regret_coeff;
  };

  auto start_solving = std::chrono::high_resolution_clock::now();
  BOOST_LOG_TRIVIAL(info) << "[CVRP] Start clustering heuristic(s).";

  std::vector<param> parameters;
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.5});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 0.3});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 1.9});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 1.4});

  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.1});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 1});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 0.1});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 1.1});

  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.6});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 1.2});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 0.5});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 1.2});

  auto P = parameters.size();

  unsigned max_nb_jobs_removal = 2;

  std::vector<raw_solution> solutions(P,
                                      raw_solution(nb_tsp,
                                                   std::vector<index_t>()));
  std::vector<solution_indicators> sol_indicators(P);

  // Split the work among threads.
  std::vector<std::vector<std::size_t>>
    thread_ranks(nb_threads, std::vector<std::size_t>());
  for (std::size_t i = 0; i < P; ++i) {
    thread_ranks[i % nb_threads].push_back(i);
  }

  auto run_solve = [&](const std::vector<std::size_t>& param_ranks) {
    for (auto rank : param_ranks) {
      auto& p = parameters[rank];
      clustering c(_input, p.type, p.init, p.regret_coeff);

      // Populate vector of TSP solutions, one per cluster.
      for (std::size_t v = 0; v < nb_tsp; ++v) {
        if (c.clusters[v].empty()) {
          continue;
        }
        tsp p(_input, c.clusters[v], v);

        solutions[rank][v] = p.solve(1)[0];
      }

      // Local search phase.
      cvrp_local_search ls(_input, solutions[rank], max_nb_jobs_removal);
      ls.run();

      // Store solution indicators.
      sol_indicators[rank] = ls.indicators();
    }
  };

  std::vector<std::thread> solving_threads;

  for (std::size_t i = 0; i < nb_threads; ++i) {
    solving_threads.emplace_back(run_solve, thread_ranks[i]);
  }

  for (auto& t : solving_threads) {
    t.join();
  }

  auto end_solving = std::chrono::high_resolution_clock::now();

  auto solving_computing_time =
    std::chrono::duration_cast<std::chrono::milliseconds>(end_solving -
                                                          start_solving)
      .count();

  auto indicators_compare = [](const auto& lhs, const auto& rhs) {
    if (lhs.unassigned < rhs.unassigned) {
      return true;
    }
    if (lhs.unassigned == rhs.unassigned) {
      if (lhs.cost < rhs.cost) {
        return true;
      }
      if (lhs.cost == rhs.cost and lhs.used_vehicles < rhs.used_vehicles) {
        return true;
      }
    }
    return false;
  };

  auto best_indic = std::min_element(sol_indicators.cbegin(),
                                     sol_indicators.cend(),
                                     indicators_compare);

  BOOST_LOG_TRIVIAL(info) << "[CVRP] Done, took " << solving_computing_time
                          << " ms.";

  return solutions[std::distance(sol_indicators.cbegin(), best_indic)];
}
