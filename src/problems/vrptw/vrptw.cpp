/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <thread>

#include "problems/vrptw/heuristics/best_insertion.h"
#include "problems/vrptw/heuristics/solomon.h"
#include "problems/vrptw/vrptw.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/tw_route.h"
#include "utils/helpers.h"

using tw_solution = std::vector<tw_route>;

vrptw::vrptw(const input& input) : vrp(input) {
}

solution vrptw::solve(unsigned exploration_level, unsigned nb_threads) const {
  struct param {
    INIT_T init;
    float regret_coeff;
  };
  std::vector<param> parameters;

  std::vector<INIT_T> inits({INIT_T::NONE,
                             INIT_T::HIGHER_AMOUNT,
                             INIT_T::EARLIEST_DEADLINE,
                             INIT_T::FURTHEST});
  for (auto init : inits) {
    for (float lambda = 0; lambda <= 3.05; lambda += 0.1) {
      parameters.push_back({init, lambda});
    }
  }

  auto P = parameters.size();
  std::vector<tw_solution> tw_solutions(P);

  // Split the work among threads.
  std::vector<std::vector<std::size_t>>
    thread_ranks(nb_threads, std::vector<std::size_t>());
  for (std::size_t i = 0; i < P; ++i) {
    thread_ranks[i % nb_threads].push_back(i);
  }

  auto run_solve = [&](const std::vector<std::size_t>& param_ranks) {
    for (auto rank : param_ranks) {
      auto& p = parameters[rank];
      tw_solutions[rank] = solomon(_input, p.init, p.regret_coeff);
    }
  };

  std::vector<std::thread> solving_threads;

  for (std::size_t i = 0; i < nb_threads; ++i) {
    solving_threads.emplace_back(run_solve, thread_ranks[i]);
  }

  for (auto& t : solving_threads) {
    t.join();
  }

  std::vector<solution> solutions;
  cost_t best_cost = std::numeric_limits<cost_t>::max();
  std::size_t best_sol_rank;
  for (const auto& tw_sol : tw_solutions) {
    solutions.push_back(format_solution(_input, tw_sol));
    if (solutions.back().summary.cost < best_cost) {
      best_cost = solutions.back().summary.cost;
      best_sol_rank = solutions.size() - 1;
    }
  }

  return solutions[best_sol_rank];
}
