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

  SolvingContext(const Input& input, unsigned nb_searches)
    : init_sol(set_init_sol<Route>(input, init_assigned)),
      vehicles_ranks(input.vehicles.size()),
      solutions(nb_searches, init_sol),
      sol_indicators(nb_searches) {

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
    }
  }

  // Check if heuristic solution has been encountered before.
  context.sol_indicators[rank] =
    utils::SolutionIndicators(input, context.solutions[rank]);

  const auto heuristic_end = utils::now();

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
    const std::vector<HeuristicParameters>& homogeneous_parameters,
    const std::vector<HeuristicParameters>& heterogeneous_parameters) const {
    const auto& parameters = (_input.has_homogeneous_locations())
                               ? homogeneous_parameters
                               : heterogeneous_parameters;
    assert(nb_searches != 0);
    nb_searches =
      std::min(nb_searches, static_cast<unsigned>(parameters.size()));

    SolvingContext<Route> context(_input, nb_searches);

    std::exception_ptr ep = nullptr;
    std::mutex ep_m;

    const auto actual_nb_threads = std::min(nb_searches, nb_threads);
    assert(actual_nb_threads <= 32);
    std::counting_semaphore<32> semaphore(actual_nb_threads);

    Timeout search_time;
    if (timeout.has_value()) {
      // Max number of solving per thread.
      const auto dv = std::div(static_cast<long>(nb_searches),
                               static_cast<long>(actual_nb_threads));
      const unsigned max_solving_number = dv.quot + ((dv.rem == 0) ? 0 : 1);
      search_time = timeout.value() / max_solving_number;
    }

    auto run_solving = [&context,
                        &semaphore,
                        &search_time,
                        &parameters,
                        &timeout,
                        &ep,
                        &ep_m,
                        depth,
                        this](const unsigned rank) {
      semaphore.acquire();
      try {
        run_single_search<Route, LocalSearch>(_input,
                                              parameters[rank],
                                              rank,
                                              depth,
                                              search_time,
                                              context);
      } catch (...) {
        const std::scoped_lock<std::mutex> lock(ep_m);
        ep = std::current_exception();
      }
      semaphore.release();
    };

    std::vector<std::jthread> solving_threads;
    solving_threads.reserve(nb_searches);

    for (unsigned i = 0; i < nb_searches; ++i) {
      solving_threads.emplace_back(run_solving, i);
    }

    for (auto& t : solving_threads) {
      t.join();
    }

    if (ep != nullptr) {
      std::rethrow_exception(ep);
    }

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

  virtual Solution solve(unsigned nb_searches,
                         unsigned depth,
                         unsigned nb_threads,
                         const Timeout& timeout) const = 0;
};

} // namespace vroom

#endif
