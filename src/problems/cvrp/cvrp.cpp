/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <numeric>
#include <thread>

#include "algorithms/heuristics/clustering.h"
#include "algorithms/heuristics/solomon.h"
#include "algorithms/local_search/local_search.h"
#include "problems/cvrp/cvrp.h"
#include "problems/cvrp/operators/cross_exchange.h"
#include "problems/cvrp/operators/exchange.h"
#include "problems/cvrp/operators/intra_cross_exchange.h"
#include "problems/cvrp/operators/intra_exchange.h"
#include "problems/cvrp/operators/intra_mixed_exchange.h"
#include "problems/cvrp/operators/intra_or_opt.h"
#include "problems/cvrp/operators/intra_relocate.h"
#include "problems/cvrp/operators/mixed_exchange.h"
#include "problems/cvrp/operators/or_opt.h"
#include "problems/cvrp/operators/relocate.h"
#include "problems/cvrp/operators/reverse_two_opt.h"
#include "problems/cvrp/operators/two_opt.h"
#include "problems/tsp/tsp.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/raw_route.h"
#include "utils/helpers.h"

namespace vroom {

using RawSolution = std::vector<RawRoute>;

using LocalSearch = ls::LocalSearch<RawRoute,
                                    cvrp::Exchange,
                                    cvrp::CrossExchange,
                                    cvrp::MixedExchange,
                                    cvrp::TwoOpt,
                                    cvrp::ReverseTwoOpt,
                                    cvrp::Relocate,
                                    cvrp::OrOpt,
                                    cvrp::IntraExchange,
                                    cvrp::IntraCrossExchange,
                                    cvrp::IntraMixedExchange,
                                    cvrp::IntraRelocate,
                                    cvrp::IntraOrOpt>;

constexpr std::array<HeuristicParameters, 32> CVRP::homogeneous_parameters;
constexpr std::array<HeuristicParameters, 32> CVRP::heterogeneous_parameters;

CVRP::CVRP(const Input& input) : VRP(input) {
}

Solution CVRP::solve(unsigned exploration_level, unsigned nb_threads) const {
  auto nb_tsp = _input.vehicles.size();

  if (nb_tsp == 1 and !_input.has_skills() and _input.amount_size() == 0) {
    // This is a plain TSP, no need to go through the trouble below.
    std::vector<Index> job_ranks(_input.jobs.size());
    std::iota(job_ranks.begin(), job_ranks.end(), 0);

    TSP p(_input, job_ranks, 0);

    return utils::format_solution(_input, p.raw_solve(0, nb_threads));
  }

  auto parameters = (_input.has_homogeneous_locations())
                      ? homogeneous_parameters
                      : heterogeneous_parameters;

  // Local search parameter.
  unsigned max_nb_jobs_removal = exploration_level;
  auto nb_init_solutions = 4 * (exploration_level + 1);
  if (exploration_level >= 4) {
    nb_init_solutions += 4;
  }
  if (exploration_level >= 5) {
    nb_init_solutions += 4;
  }
  assert(nb_init_solutions <= parameters.size());

  std::vector<RawSolution> solutions(nb_init_solutions, RawSolution(nb_tsp));
  std::vector<utils::SolutionIndicators> sol_indicators(nb_init_solutions);

  // Split the work among threads.
  std::vector<std::vector<std::size_t>>
    thread_ranks(nb_threads, std::vector<std::size_t>());
  for (std::size_t i = 0; i < nb_init_solutions; ++i) {
    thread_ranks[i % nb_threads].push_back(i);
  }

  auto run_solve = [&](const std::vector<std::size_t>& param_ranks) {
    for (auto rank : param_ranks) {
      auto& p = parameters[rank];

      if (p.is_clustering) {
        heuristics::Clustering c(_input, p.type, p.init, p.regret_coeff);

        // Populate vector of TSP solutions, one per cluster.
        for (std::size_t v = 0; v < nb_tsp; ++v) {
          if (c.clusters[v].empty()) {
            continue;
          }
          TSP p(_input, c.clusters[v], v);

          solutions[rank][v] = p.raw_solve(0, 1)[0];
        }
      } else {
        switch (p.heuristic) {
        case HEURISTIC::BASIC:
          solutions[rank] =
            heuristics::basic<RawSolution>(_input, p.init, p.regret_coeff);
          break;
        case HEURISTIC::DYNAMIC:
          solutions[rank] =
            heuristics::dynamic_vehicle_choice<RawSolution>(_input,
                                                            p.init,
                                                            p.regret_coeff);
          break;
        }
      }

      // Local search phase.
      LocalSearch ls(_input, solutions[rank], max_nb_jobs_removal);
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

  return utils::format_solution(_input,
                                solutions[std::distance(sol_indicators.cbegin(),
                                                        best_indic)]);
}

} // namespace vroom
