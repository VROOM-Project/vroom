#ifndef VRP_H
#define VRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <mutex>
#include <numeric>
#include <ranges>
#include <set>
#include <thread>

#include "algorithms/heuristics/heuristics.h"
#include "algorithms/local_search/local_search.h"
#include "structures/vroom/eval.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution/solution.h"

#ifdef LOG_LS
#include "algorithms/local_search/log_local_search.h"
#include "utils/output_json.h"
#endif

namespace vroom {

template <class Route>
std::vector<Route> set_init_sol(const Input& input,
                                std::unordered_set<Index>& init_assigned) {
  std::vector<Route> init_sol;
  init_sol.reserve(input.vehicles.size());

  for (Index v = 0; v < input.vehicles.size(); ++v) {
    init_sol.emplace_back(input, v, input.zero_amount().size());
  }

  if (input.has_initial_routes()) {
    heuristics::set_initial_routes<Route>(input, init_sol, init_assigned);
  }

  return init_sol;
}

template <class Route> struct SolvingContext {
  std::unordered_set<Index> init_assigned;
  const std::vector<Route> init_sol;
  std::set<Index> unassigned;
  std::vector<Index> vehicles_ranks;
  std::vector<std::vector<Route>> solutions;
  std::vector<utils::SolutionIndicators> sol_indicators;

  std::set<utils::SolutionIndicators> heuristic_indicators;
  std::mutex heuristic_indicators_m;

#ifdef LOG_LS_OPERATORS
  std::vector<std::array<ls::OperatorStats, OperatorName::MAX>> ls_stats;
#endif

#ifdef LOG_LS
  std::vector<ls::log::Dump> ls_dumps;
#endif

  SolvingContext(const Input& input, unsigned nb_searches)
    : init_sol(set_init_sol<Route>(input, init_assigned)),
      vehicles_ranks(input.vehicles.size()),
      solutions(nb_searches, init_sol),
      sol_indicators(nb_searches)
#ifdef LOG_LS_OPERATORS
      ,
      ls_stats(nb_searches)
#endif
  {

    // Deduce unassigned jobs from initial solution.
    std::ranges::copy_if(std::views::iota(0u, input.jobs.size()),
                         std::inserter(unassigned, unassigned.begin()),
                         [this](const Index j) {
                           return !init_assigned.contains(j);
                         });

    // Heuristics will operate on all vehicles.
    std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);
  }

  bool heuristic_solution_already_found(unsigned rank) {
    assert(rank < sol_indicators.size());
    const std::scoped_lock<std::mutex> lock(heuristic_indicators_m);
    const auto [dummy, insertion_ok] =
      heuristic_indicators.insert(sol_indicators[rank]);

    return !insertion_ok;
  }
};

template <class Route, class LocalSearch>
void run_single_search(const Input& input,
                       const HeuristicParameters& p,
                       const unsigned rank,
                       const unsigned depth,
                       const Timeout& search_time,
                       SolvingContext<Route>& context) {
  const auto heuristic_start = utils::now();

#ifdef LOG_LS
  context.ls_dumps[rank].steps.emplace_back(heuristic_start,
                                            ls::log::EVENT::START,
                                            OperatorName::MAX);
#endif

  Eval h_eval;
  switch (p.heuristic) {
  case HEURISTIC::BASIC:
    h_eval = heuristics::basic<Route>(input,
                                      context.solutions[rank],
                                      context.unassigned,
                                      context.vehicles_ranks,
                                      p.init,
                                      p.regret_coeff,
                                      p.sort);
    break;
  case HEURISTIC::DYNAMIC:
    h_eval = heuristics::dynamic_vehicle_choice<Route>(input,
                                                       context.solutions[rank],
                                                       context.unassigned,
                                                       context.vehicles_ranks,
                                                       p.init,
                                                       p.regret_coeff,
                                                       p.sort);
    break;
  }

  if (!input.has_homogeneous_costs() && p.sort == SORT::AVAILABILITY) {
    // Worth trying another vehicle ordering scheme in
    // heuristic.
    std::vector<Route> other_sol = context.init_sol;

    Eval h_other_eval;
    switch (p.heuristic) {
    case HEURISTIC::BASIC:
      h_other_eval = heuristics::basic<Route>(input,
                                              other_sol,
                                              context.unassigned,
                                              context.vehicles_ranks,
                                              p.init,
                                              p.regret_coeff,
                                              SORT::COST);
      break;
    case HEURISTIC::DYNAMIC:
      h_other_eval =
        heuristics::dynamic_vehicle_choice<Route>(input,
                                                  other_sol,
                                                  context.unassigned,
                                                  context.vehicles_ranks,
                                                  p.init,
                                                  p.regret_coeff,
                                                  SORT::COST);
      break;
    }

    if (h_other_eval < h_eval) {
      context.solutions[rank] = std::move(other_sol);
#ifdef LOG_LS
      context.ls_dumps[rank].heuristic_parameters.sort = SORT::COST;
#endif
    }
  }

  // Check if heuristic solution has been encountered before.
  context.sol_indicators[rank] =
    utils::SolutionIndicators(input, context.solutions[rank]);

  const auto heuristic_end = utils::now();

#ifdef LOG_LS
  context.ls_dumps[rank]
    .steps.emplace_back(heuristic_end,
                        ls::log::EVENT::HEURISTIC,
                        OperatorName::MAX,
                        context.sol_indicators[rank],
                        utils::format_solution(input, context.solutions[rank]));
#endif

  if (context.heuristic_solution_already_found(rank)) {
    // Duplicate heuristic solution, so skip local search.
    return;
  }

  Timeout ls_search_time;
  if (search_time.has_value()) {
    const auto heuristic_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(heuristic_end -
                                                            heuristic_start);

    if (search_time.value() <= heuristic_time) {
      // No time left for local search!
      return;
    }

    ls_search_time = search_time.value() - heuristic_time;
  }

  // Local search phase.
  LocalSearch ls(input, context.solutions[rank], depth, ls_search_time);
  ls.run();

  // Store solution indicators.
  context.sol_indicators[rank] = ls.indicators();
#ifdef LOG_LS_OPERATORS
  context.ls_stats[rank] = ls.get_stats();
#endif
#ifdef LOG_LS
  auto ls_steps = ls.get_steps();

  assert(context.ls_dumps[rank].steps.size() == 2);
  context.ls_dumps[rank].steps.reserve(2 + ls_steps.size());

  std::ranges::move(ls_steps, std::back_inserter(context.ls_dumps[rank].steps));
#endif
}

class VRP {
  // Abstract class describing a VRP (vehicle routing problem).
protected:
  const Input& _input;

  template <class Route, class LocalSearch>
  Solution solve(
    unsigned nb_searches,
    const unsigned depth,
    const unsigned nb_threads,
    const Timeout& timeout,
    const std::vector<HeuristicParameters>& h_param,
    const std::vector<HeuristicParameters>& homogeneous_parameters,
    const std::vector<HeuristicParameters>& heterogeneous_parameters) const {
    // Use vector of parameters when passed for debugging, else use
    // predefined parameter set.
    const auto& parameters = (!h_param.empty()) ? h_param
                             : (_input.has_homogeneous_locations())
                               ? homogeneous_parameters
                               : heterogeneous_parameters;
    assert(nb_searches != 0);
    nb_searches =
      std::min(nb_searches, static_cast<unsigned>(parameters.size()));

    SolvingContext<Route> context(_input, nb_searches);

    // Split the heuristic parameters among threads.
    std::vector<std::vector<std::size_t>>
      thread_ranks(nb_threads, std::vector<std::size_t>());
    for (std::size_t i = 0; i < nb_searches; ++i) {
      thread_ranks[i % nb_threads].push_back(i);

#ifdef LOG_LS
      context.ls_dumps.push_back({parameters[i], {}});
#endif
    }

    std::exception_ptr ep = nullptr;
    std::mutex ep_m;

    auto run_solving =
      [&context, &parameters, &timeout, &ep, &ep_m, depth, this](
        const std::vector<std::size_t>& param_ranks) {
        try {
          // Decide time allocated for each search.
          Timeout search_time;
          if (timeout.has_value()) {
            search_time = timeout.value() / param_ranks.size();
          }

          for (auto rank : param_ranks) {
            run_single_search<Route, LocalSearch>(_input,
                                                  parameters[rank],
                                                  rank,
                                                  depth,
                                                  search_time,
                                                  context);
          }
        } catch (...) {
          const std::scoped_lock<std::mutex> lock(ep_m);
          ep = std::current_exception();
        }
      };

    std::vector<std::jthread> solving_threads;
    solving_threads.reserve(nb_threads);

    for (const auto& param_ranks : thread_ranks) {
      if (!param_ranks.empty()) {
        solving_threads.emplace_back(run_solving, param_ranks);
      }
    }

    for (auto& t : solving_threads) {
      t.join();
    }

    if (ep != nullptr) {
      std::rethrow_exception(ep);
    }

#ifdef LOG_LS_OPERATORS
    utils::log_LS_operators(context.ls_stats);
#endif

#ifdef LOG_LS
    io::write_LS_logs_to_json(context.ls_dumps);
#endif

    auto best_indic = std::min_element(context.sol_indicators.cbegin(),
                                       context.sol_indicators.cend());

    return utils::
      format_solution(_input,
                      context.solutions[std::distance(context.sol_indicators
                                                        .cbegin(),
                                                      best_indic)]);
  }

public:
  explicit VRP(const Input& input);

  virtual ~VRP();

  virtual Solution
  solve(unsigned nb_searches,
        unsigned depth,
        unsigned nb_threads,
        const Timeout& timeout,
        const std::vector<HeuristicParameters>& h_param) const = 0;
};

} // namespace vroom

#endif
