/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <mutex>
#include <thread>

#include "algorithms/heuristics/heuristics.h"
#include "algorithms/local_search/local_search.h"
#include "problems/cvrp/cvrp.h"
#include "problems/cvrp/operators/cross_exchange.h"
#include "problems/cvrp/operators/intra_cross_exchange.h"
#include "problems/cvrp/operators/intra_exchange.h"
#include "problems/cvrp/operators/intra_mixed_exchange.h"
#include "problems/cvrp/operators/intra_or_opt.h"
#include "problems/cvrp/operators/intra_relocate.h"
#include "problems/cvrp/operators/mixed_exchange.h"
#include "problems/cvrp/operators/or_opt.h"
#include "problems/cvrp/operators/pd_shift.h"
#include "problems/cvrp/operators/relocate.h"
#include "problems/cvrp/operators/reverse_two_opt.h"
#include "problems/cvrp/operators/route_exchange.h"
#include "problems/cvrp/operators/swap_star.h"
#include "problems/cvrp/operators/two_opt.h"
#include "problems/cvrp/operators/unassigned_exchange.h"
#include "problems/tsp/tsp.h"
#include "utils/helpers.h"

namespace vroom {

using RawSolution = std::vector<RawRoute>;

using LocalSearch = ls::LocalSearch<RawRoute,
                                    cvrp::UnassignedExchange,
                                    cvrp::SwapStar,
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
                                    cvrp::IntraOrOpt,
                                    cvrp::PDShift,
                                    cvrp::RouteExchange>;

const std::vector<HeuristicParameters> CVRP::homogeneous_parameters =
  {HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.3),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.5),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.3),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.7),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.3),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 2.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.2),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.6),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.9),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 2.4),

   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.8),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.4),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 1.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 2.4),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.9),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 1.5),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.5),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 1.3),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1.5),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.1),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.4),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 1.6),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.6),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.5)};

const std::vector<HeuristicParameters> CVRP::heterogeneous_parameters =
  {HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.4),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.3),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.3),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.6),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.2),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.9),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.3),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.4),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.5),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 1.4),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 2.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.8),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.6),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.8),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.4),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.5),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.6),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.7),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 1.2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.5),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.7),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 1.7),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 2.2)};

CVRP::CVRP(const Input& input) : VRP(input) {
}

Solution CVRP::solve(unsigned exploration_level,
                     unsigned nb_threads,
                     const Timeout& timeout,
                     const std::vector<HeuristicParameters>& h_param) const {
  if (_input.vehicles.size() == 1 and !_input.has_skills() and
      _input.zero_amount().size() == 0 and !_input.has_shipments() and
      (_input.jobs.size() <= _input.vehicles[0].max_tasks) and
      _input.vehicles[0].steps.empty()) {
    // This is a plain TSP, no need to go through the trouble below.
    std::vector<Index> job_ranks(_input.jobs.size());
    std::iota(job_ranks.begin(), job_ranks.end(), 0);

    TSP p(_input, job_ranks, 0);

    RawRoute r(_input, 0);
    r.set_route(_input, p.raw_solve(nb_threads, timeout));

    return utils::format_solution(_input, {r});
  }

  // Use vector of parameters when passed for debugging, else use
  // predefined parameter set.
  const auto& parameters = (!h_param.empty())
                             ? h_param
                             : (_input.has_homogeneous_locations())
                                 ? homogeneous_parameters
                                 : heterogeneous_parameters;
  unsigned max_nb_jobs_removal = exploration_level;
  unsigned nb_init_solutions = h_param.size();

  if (nb_init_solutions == 0) {
    // Local search parameter.
    nb_init_solutions = 4 * (exploration_level + 1);
    if (exploration_level >= 4) {
      nb_init_solutions += 4;
    }
    if (exploration_level >= 5) {
      nb_init_solutions += 4;
    }
  }
  assert(nb_init_solutions <= parameters.size());

  std::vector<RawSolution> solutions(nb_init_solutions);
  std::vector<utils::SolutionIndicators> sol_indicators(nb_init_solutions);

  // Split the work among threads.
  std::vector<std::vector<std::size_t>>
    thread_ranks(nb_threads, std::vector<std::size_t>());
  for (std::size_t i = 0; i < nb_init_solutions; ++i) {
    thread_ranks[i % nb_threads].push_back(i);
  }

  std::exception_ptr ep = nullptr;
  std::mutex ep_m;

  auto run_solve = [&](const std::vector<std::size_t>& param_ranks) {
    try {
      // Decide time allocated for each search.
      Timeout search_time;
      if (timeout.has_value()) {
        search_time = timeout.value() / param_ranks.size();
      }

      for (auto rank : param_ranks) {
        auto& p = parameters[rank];

        switch (p.heuristic) {
        case HEURISTIC::INIT_ROUTES:
          solutions[rank] = heuristics::initial_routes<RawSolution>(_input);
          break;
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

        // Local search phase.
        LocalSearch ls(_input,
                       solutions[rank],
                       max_nb_jobs_removal,
                       search_time);
        ls.run();

        // Store solution indicators.
        sol_indicators[rank] = ls.indicators();
      }
    } catch (...) {
      ep_m.lock();
      ep = std::current_exception();
      ep_m.unlock();
    }
  };

  std::vector<std::thread> solving_threads;

  for (const auto& param_ranks : thread_ranks) {
    if (!param_ranks.empty()) {
      solving_threads.emplace_back(run_solve, param_ranks);
    }
  }

  for (auto& t : solving_threads) {
    t.join();
  }

  if (ep != nullptr) {
    std::rethrow_exception(ep);
  }

  auto best_indic =
    std::min_element(sol_indicators.cbegin(), sol_indicators.cend());

  return utils::format_solution(_input,
                                solutions[std::distance(sol_indicators.cbegin(),
                                                        best_indic)]);
}

} // namespace vroom
