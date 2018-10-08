/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <thread>

#include "problems/vrptw/heuristics/solomon.h"
#include "problems/vrptw/local_search/local_search.h"
#include "problems/vrptw/vrptw.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/tw_route.h"
#include "utils/helpers.h"

using tw_solution = std::vector<tw_route>;

constexpr std::array<h_param, 24> vrptw::homogeneous_parameters;

vrptw::vrptw(const input& input) : vrp(input) {
}

solution vrptw::solve(unsigned exploration_level, unsigned nb_threads) const {
  // Local search parameter.
  unsigned max_nb_jobs_removal = exploration_level;
  // Number of initial solutions to consider.
  auto P = 4 * (exploration_level + 1);
  assert(P <= homogeneous_parameters.size());

  std::vector<tw_solution> tw_solutions(P);
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
      switch (p.heuristic) {
      case HEURISTIC_T::BASIC:
        tw_solutions[rank] =
          vrptw_basic_heuristic(_input, p.init, p.regret_coeff);
        break;
      case HEURISTIC_T::DYNAMIC:
        tw_solutions[rank] =
          vrptw_dynamic_vehicle_choice_heuristic(_input,
                                                 p.init,
                                                 p.regret_coeff);
        break;
      }

      // Local search phase.
      vrptw_local_search ls(_input, tw_solutions[rank], max_nb_jobs_removal);
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
                         tw_solutions[std::distance(sol_indicators.cbegin(),
                                                    best_indic)]);
}
