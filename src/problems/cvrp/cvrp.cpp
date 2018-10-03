/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>
#include <thread>

#include "problems/cvrp/cvrp.h"
#include "problems/cvrp/heuristics/solomon.h"
#include "problems/cvrp/local_search/local_search.h"
#include "problems/tsp/tsp.h"
#include "structures/vroom/input/input.h"
#include "utils/helpers.h"

constexpr std::array<h_param, 24> cvrp::homogeneous_parameters;

cvrp::cvrp(const input& input) : vrp(input) {
}

solution cvrp::solve(unsigned exploration_level, unsigned nb_threads) const {
  auto nb_tsp = _input._vehicles.size();

  if (nb_tsp == 1 and !_input.has_skills() and _input.amount_size() == 0) {
    // This is a plain TSP, no need to go through the trouble below.
    std::vector<index_t> job_ranks(_input._jobs.size());
    std::iota(job_ranks.begin(), job_ranks.end(), 0);

    tsp p(_input, job_ranks, 0);

    return format_solution(_input, p.raw_solve(0, nb_threads));
  }

  // Local search parameter.
  unsigned max_nb_jobs_removal = exploration_level;
  // Number of initial solutions to consider.
  auto P = 4 * (exploration_level + 1);
  assert(P <= homogeneous_parameters.size());

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
      auto& p = homogeneous_parameters[rank];

      if (p.is_clustering) {
        clustering c(_input, p.type, p.init, p.regret_coeff);

        // Populate vector of TSP solutions, one per cluster.
        for (std::size_t v = 0; v < nb_tsp; ++v) {
          if (c.clusters[v].empty()) {
            continue;
          }
          tsp p(_input, c.clusters[v], v);

          solutions[rank][v] = p.raw_solve(0, 1)[0];
        }
      } else {
        switch (p.heuristic) {
        case HEURISTIC_T::BASIC:
          solutions[rank] =
            cvrp_basic_heuristic(_input, p.init, p.regret_coeff);
          break;
        case HEURISTIC_T::DYNAMIC:
          solutions[rank] =
            cvrp_dynamic_vehicle_choice_heuristic(_input,
                                                  p.init,
                                                  p.regret_coeff);
          break;
        }
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

  auto best_indic =
    std::min_element(sol_indicators.cbegin(), sol_indicators.cend());

  return format_solution(_input,
                         solutions[std::distance(sol_indicators.cbegin(),
                                                 best_indic)]);
}
