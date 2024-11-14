#ifndef VRP_H
#define VRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
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

class VRP {
  // Abstract class describing a VRP (vehicle routing problem).
protected:
  const Input& _input;

  template <class Route, class LocalSearch>
  Solution solve(
    unsigned nb_searches,
    unsigned depth,
    unsigned nb_threads,
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

    // Build initial solution to be filled by heuristics. Solution is
    // empty at first but populated with input data if provided.
    std::vector<Route> init_sol;
    init_sol.reserve(_input.vehicles.size());

    for (Index v = 0; v < _input.vehicles.size(); ++v) {
      init_sol.emplace_back(_input, v, _input.zero_amount().size());
    }

    std::unordered_set<Index> init_assigned;
    if (_input.has_initial_routes()) {
      init_assigned = heuristics::set_initial_routes<Route>(_input, init_sol);
    }

    std::vector<std::vector<Route>> solutions(nb_searches, init_sol);

#ifdef LOG_LS
    std::vector<ls::log::Dump> ls_dumps;
    ls_dumps.reserve(nb_searches);
#endif

    // Heuristics operate on all assigned jobs.
    std::set<Index> unassigned;
    std::ranges::copy_if(std::views::iota(0u, _input.jobs.size()),
                         std::inserter(unassigned, unassigned.begin()),
                         [&init_assigned](const Index j) {
                           return !init_assigned.contains(j);
                         });

    // Heuristics operate on all vehicles.
    std::vector<Index> vehicles_ranks(_input.vehicles.size());
    std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);

    // Split the heuristic parameters among threads.
    std::vector<std::vector<std::size_t>>
      thread_ranks(nb_threads, std::vector<std::size_t>());
    for (std::size_t i = 0; i < nb_searches; ++i) {
      thread_ranks[i % nb_threads].push_back(i);

#ifdef LOG_LS
      ls_dumps.push_back({parameters[i], {}});
#endif
    }

    std::exception_ptr ep = nullptr;
    std::mutex ep_m;

    auto run_heuristics = [&](const std::vector<std::size_t>& param_ranks) {
      try {
        for (auto rank : param_ranks) {
          const auto& p = parameters[rank];

          Eval h_eval;
          switch (p.heuristic) {
          case HEURISTIC::BASIC:
            h_eval = heuristics::basic<Route>(_input,
                                              solutions[rank],
                                              unassigned,
                                              vehicles_ranks,
                                              p.init,
                                              p.regret_coeff,
                                              p.sort);
            break;
          case HEURISTIC::DYNAMIC:
            h_eval = heuristics::dynamic_vehicle_choice<Route>(_input,
                                                               solutions[rank],
                                                               unassigned,
                                                               vehicles_ranks,
                                                               p.init,
                                                               p.regret_coeff,
                                                               p.sort);
            break;
          }

          if (!_input.has_homogeneous_costs() && h_param.empty() &&
              p.sort == SORT::AVAILABILITY) {
            // Worth trying another vehicle ordering scheme in
            // heuristic.
            std::vector<Route> other_sol = init_sol;

            Eval h_other_eval;
            switch (p.heuristic) {
            case HEURISTIC::BASIC:
              h_other_eval = heuristics::basic<Route>(_input,
                                                      other_sol,
                                                      unassigned,
                                                      vehicles_ranks,
                                                      p.init,
                                                      p.regret_coeff,
                                                      SORT::COST);
              break;
            case HEURISTIC::DYNAMIC:
              h_other_eval =
                heuristics::dynamic_vehicle_choice<Route>(_input,
                                                          other_sol,
                                                          unassigned,
                                                          vehicles_ranks,
                                                          p.init,
                                                          p.regret_coeff,
                                                          SORT::COST);
              break;
            }

            if (h_other_eval < h_eval) {
              solutions[rank] = std::move(other_sol);
#ifdef LOG_LS
              ls_dumps[rank].heuristic_parameters.sort = SORT::COST;
#endif
            }
          }
        }
      } catch (...) {
        std::scoped_lock<std::mutex> lock(ep_m);
        ep = std::current_exception();
      }
    };

    std::vector<std::jthread> heuristics_threads;
    heuristics_threads.reserve(nb_threads);

    for (const auto& param_ranks : thread_ranks) {
      if (!param_ranks.empty()) {
        heuristics_threads.emplace_back(run_heuristics, param_ranks);
      }
    }

    for (auto& t : heuristics_threads) {
      t.join();
    }

    if (ep != nullptr) {
      std::rethrow_exception(ep);
    }

    // Filter out duplicate heuristics solutions.
    std::set<utils::SolutionIndicators> unique_indicators;
    std::vector<unsigned> to_remove;
    to_remove.reserve(solutions.size());

    for (unsigned i = 0; i < solutions.size(); ++i) {
      const auto result = unique_indicators.emplace(_input, solutions[i]);
      if (!result.second) {
        // No insertion means an equivalent solution already exists.
        to_remove.push_back(i);
      }
    }

    for (auto remove_rank = to_remove.rbegin(); remove_rank != to_remove.rend();
         remove_rank++) {
      solutions.erase(solutions.begin() + *remove_rank);
#ifdef LOG_LS
      ls_dumps.erase(ls_dumps.begin() + *remove_rank);
#endif
    }

    // Split local searches across threads.
    unsigned nb_solutions = solutions.size();
    std::vector<utils::SolutionIndicators> sol_indicators(nb_solutions);
#ifdef LOG_LS_OPERATORS
    std::vector<std::array<ls::OperatorStats, OperatorName::MAX>> ls_stats(
      nb_solutions);
#endif

    std::ranges::fill(thread_ranks, std::vector<std::size_t>());
    for (std::size_t i = 0; i < nb_solutions; ++i) {
      thread_ranks[i % nb_threads].push_back(i);
    }

    auto run_ls = [&](const std::vector<std::size_t>& sol_ranks) {
      try {
        // Decide time allocated for each search.
        Timeout search_time;
        if (timeout.has_value()) {
          search_time = timeout.value() / sol_ranks.size();
        }

        for (auto rank : sol_ranks) {
          // Local search phase.
          LocalSearch ls(_input, solutions[rank], depth, search_time);
          ls.run();

          // Store solution indicators.
          sol_indicators[rank] = ls.indicators();
#ifdef LOG_LS_OPERATORS
          ls_stats[rank] = ls.get_stats();
#endif
#ifdef LOG_LS
          ls_dumps[rank].steps = ls.get_steps();
#endif
        }
      } catch (...) {
        std::scoped_lock<std::mutex> lock(ep_m);
        ep = std::current_exception();
      }
    };

    std::vector<std::jthread> ls_threads;
    ls_threads.reserve(nb_threads);

    for (const auto& sol_ranks : thread_ranks) {
      if (!sol_ranks.empty()) {
        ls_threads.emplace_back(run_ls, sol_ranks);
      }
    }

    for (auto& t : ls_threads) {
      t.join();
    }

    if (ep != nullptr) {
      std::rethrow_exception(ep);
    }

#ifdef LOG_LS_OPERATORS
    utils::log_LS_operators(ls_stats);
#endif

#ifdef LOG_LS
    io::write_LS_logs_to_json(ls_dumps);
#endif

    auto best_indic =
      std::min_element(sol_indicators.cbegin(), sol_indicators.cend());

    return utils::format_solution(_input,
                                  solutions[std::distance(sol_indicators
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
