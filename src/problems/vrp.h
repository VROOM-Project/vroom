#ifndef VRP_H
#define VRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <mutex>
#include <numeric>
#include <set>
#include <thread>

#include "algorithms/heuristics/heuristics.h"
#include "algorithms/local_search/local_search.h"
#include "structures/vroom/eval.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution/solution.h"

namespace vroom {

class VRP {
  // Abstract class describing a VRP (vehicle routing problem).
protected:
  const Input& _input;

  template <class Route, class LocalSearch>
  Solution solve(
    unsigned exploration_level,
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
    unsigned max_nb_jobs_removal = exploration_level;
    unsigned nb_init_solutions = h_param.size();

    if (nb_init_solutions == 0) {
      // Local search parameter.
      nb_init_solutions = 4 * (exploration_level + 1);
      if (exploration_level >= 4) {
        nb_init_solutions += 4;
      }
      if (exploration_level == MAX_EXPLORATION_LEVEL) {
        nb_init_solutions += 4;
      }
    }
    assert(nb_init_solutions <= parameters.size());

    // Build empty solutions to be filled by heuristics.
    std::vector<Route> empty_sol;
    empty_sol.reserve(_input.vehicles.size());

    for (Index v = 0; v < _input.vehicles.size(); ++v) {
      empty_sol.emplace_back(_input, v, _input.zero_amount().size());
    }

    std::vector<std::vector<Route>> solutions(nb_init_solutions, empty_sol);

    // Heuristics operate on all jobs.
    std::vector<Index> jobs_ranks(_input.jobs.size());
    std::iota(jobs_ranks.begin(), jobs_ranks.end(), 0);

    // Heuristics operate on all vehicles.
    std::vector<Index> vehicles_ranks(_input.vehicles.size());
    std::iota(vehicles_ranks.begin(), vehicles_ranks.end(), 0);

    // Split the heuristic parameters among threads.
    std::vector<std::vector<std::size_t>>
      thread_ranks(nb_threads, std::vector<std::size_t>());
    for (std::size_t i = 0; i < nb_init_solutions; ++i) {
      thread_ranks[i % nb_threads].push_back(i);
    }

    std::exception_ptr ep = nullptr;
    std::mutex ep_m;

    auto run_heuristics = [&](const std::vector<std::size_t>& param_ranks) {
      try {
        for (auto rank : param_ranks) {
          const auto& p = parameters[rank];

          Eval h_eval;
          switch (p.heuristic) {
          case HEURISTIC::INIT_ROUTES:
            heuristics::initial_routes<Route>(_input, solutions[rank]);
            break;
          case HEURISTIC::BASIC:
            h_eval = heuristics::basic<Route>(_input,
                                              solutions[rank],
                                              jobs_ranks.cbegin(),
                                              jobs_ranks.cend(),
                                              vehicles_ranks.cbegin(),
                                              vehicles_ranks.cend(),
                                              p.init,
                                              p.regret_coeff,
                                              p.sort);
            break;
          case HEURISTIC::DYNAMIC:
            h_eval =
              heuristics::dynamic_vehicle_choice<Route>(_input,
                                                        solutions[rank],
                                                        jobs_ranks.cbegin(),
                                                        jobs_ranks.cend(),
                                                        vehicles_ranks.cbegin(),
                                                        vehicles_ranks.cend(),
                                                        p.init,
                                                        p.regret_coeff,
                                                        p.sort);
            break;
          }

          if (!_input.has_homogeneous_costs() &&
              p.heuristic != HEURISTIC::INIT_ROUTES && h_param.empty() &&
              p.sort == SORT::AVAILABILITY) {
            // Worth trying another vehicle ordering scheme in
            // heuristic.
            std::vector<Route> other_sol = empty_sol;

            Eval h_other_eval;
            switch (p.heuristic) {
            case HEURISTIC::INIT_ROUTES:
              assert(false);
              break;
            case HEURISTIC::BASIC:
              h_other_eval = heuristics::basic<Route>(_input,
                                                      other_sol,
                                                      jobs_ranks.cbegin(),
                                                      jobs_ranks.cend(),
                                                      vehicles_ranks.cbegin(),
                                                      vehicles_ranks.cend(),
                                                      p.init,
                                                      p.regret_coeff,
                                                      SORT::COST);
              break;
            case HEURISTIC::DYNAMIC:
              h_other_eval =
                heuristics::dynamic_vehicle_choice<Route>(_input,
                                                          other_sol,
                                                          jobs_ranks.cbegin(),
                                                          jobs_ranks.cend(),
                                                          vehicles_ranks
                                                            .cbegin(),
                                                          vehicles_ranks.cend(),
                                                          p.init,
                                                          p.regret_coeff,
                                                          SORT::COST);
              break;
            }

            if (h_other_eval < h_eval) {
              solutions[rank] = std::move(other_sol);
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
    std::set<utils::SolutionIndicators<Route>> unique_indicators;
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
    }

    // Split local searches across threads.
    unsigned nb_solutions = solutions.size();
    std::vector<utils::SolutionIndicators<Route>> sol_indicators(nb_solutions);
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
          LocalSearch ls(_input,
                         solutions[rank],
                         max_nb_jobs_removal,
                         search_time);
          ls.run();

          // Store solution indicators.
          sol_indicators[rank] = ls.indicators();
#ifdef LOG_LS_OPERATORS
          ls_stats[rank] = ls.get_stats();
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
  solve(unsigned exploration_level,
        unsigned nb_threads,
        const Timeout& timeout,
        const std::vector<HeuristicParameters>& h_param) const = 0;
};

} // namespace vroom

#endif
