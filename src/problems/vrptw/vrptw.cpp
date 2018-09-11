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

vrptw::vrptw(const input& input) : vrp(input) {
}

solution vrptw::solve(unsigned exploration_level, unsigned nb_threads) const {
  // Heuristic flavor.
  enum class HEURISTIC_T { BASIC, DYNAMIC };

  struct param {
    HEURISTIC_T heuristic;
    INIT_T init;
    float regret_coeff;
  };

  HEURISTIC_T h_flavor = HEURISTIC_T::BASIC;
  if (!_input.has_homogeneous_locations()) {
    h_flavor = HEURISTIC_T::DYNAMIC;
  }
  std::vector<param> parameters({{h_flavor, INIT_T::NONE, 1},
                                 {h_flavor, INIT_T::NONE, 1.3},
                                 {h_flavor, INIT_T::FURTHEST, 0.4},
                                 {h_flavor, INIT_T::FURTHEST, 0.6}});

  if (exploration_level > 0) {
    parameters.push_back({h_flavor, INIT_T::NONE, 1.2});
    parameters.push_back({h_flavor, INIT_T::EARLIEST_DEADLINE, 0.2});
    parameters.push_back({h_flavor, INIT_T::EARLIEST_DEADLINE, 2.1});
    parameters.push_back({h_flavor, INIT_T::FURTHEST, 0.5});
  }

  if (exploration_level > 1) {
    parameters.push_back({h_flavor, INIT_T::HIGHER_AMOUNT, 2.1});
    parameters.push_back({h_flavor, INIT_T::EARLIEST_DEADLINE, 0.6});
    parameters.push_back({h_flavor, INIT_T::FURTHEST, 0.7});
    parameters.push_back({h_flavor, INIT_T::FURTHEST, 1.1});
  }

  auto P = parameters.size();
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
      auto& p = parameters[rank];
      switch (p.heuristic) {
      case HEURISTIC_T::BASIC:
        tw_solutions[rank] = basic_heuristic(_input, p.init, p.regret_coeff);
        break;
      case HEURISTIC_T::DYNAMIC:
        tw_solutions[rank] =
          dynamic_vehicle_choice_heuristic(_input, p.init, p.regret_coeff);
        break;
      }

      // Local search phase.
      vrptw_local_search ls(_input, tw_solutions[rank]);
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
