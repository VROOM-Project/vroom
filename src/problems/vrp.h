#ifndef VRP_H
#define VRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <mutex>
#include <set>
#include <thread>

#include "algorithms/heuristics/heuristics.h"
#include "algorithms/local_search/local_search.h"
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
      if (exploration_level >= 5) {
        nb_init_solutions += 4;
      }
    }
    assert(nb_init_solutions <= parameters.size());

    std::vector<std::vector<Route>> solutions(nb_init_solutions);

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
          auto& p = parameters[rank];

          switch (p.heuristic) {
          case HEURISTIC::INIT_ROUTES:
            solutions[rank] =
              heuristics::initial_routes<std::vector<Route>>(_input);
            break;
          case HEURISTIC::BASIC:
            solutions[rank] =
              heuristics::basic<std::vector<Route>>(_input,
                                                    p.init,
                                                    p.regret_coeff,
                                                    p.sort);
            break;
          case HEURISTIC::DYNAMIC:
            solutions[rank] = heuristics::dynamic_vehicle_choice<
              std::vector<Route>>(_input, p.init, p.regret_coeff, p.sort);
            break;
          }

          if (!_input.has_homogeneous_costs() and
              p.heuristic != HEURISTIC::INIT_ROUTES and h_param.empty() and
              p.sort == SORT::CAPACITY) {
            // Worth trying another vehicle ordering scheme in
            // heuristic.
            std::vector<Route> other_sol;

            switch (p.heuristic) {
            case HEURISTIC::INIT_ROUTES:
              assert(false);
              break;
            case HEURISTIC::BASIC:
              other_sol = heuristics::basic<std::vector<Route>>(_input,
                                                                p.init,
                                                                p.regret_coeff,
                                                                SORT::COST);
              break;
            case HEURISTIC::DYNAMIC:
              other_sol = heuristics::dynamic_vehicle_choice<
                std::vector<Route>>(_input, p.init, p.regret_coeff, SORT::COST);
              break;
            }

            Eval eval, other_eval;
            for (Index v = 0; v < _input.vehicles.size(); ++v) {
              eval += utils::route_eval_for_vehicle(_input,
                                                    v,
                                                    solutions[rank][v].route);
              other_eval +=
                utils::route_eval_for_vehicle(_input, v, other_sol[v].route);
            }
            if (other_eval < eval) {
              solutions[rank] = std::move(other_sol);
            }
          }
        }
      } catch (...) {
        ep_m.lock();
        ep = std::current_exception();
        ep_m.unlock();
      }
    };

    std::vector<std::thread> heuristics_threads;

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

    std::fill(thread_ranks.begin(),
              thread_ranks.end(),
              std::vector<std::size_t>());
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
        ep_m.lock();
        ep = std::current_exception();
        ep_m.unlock();
      }
    };

    std::vector<std::thread> ls_threads;

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
  VRP(const Input& input);

  virtual ~VRP();

  virtual Solution
  solve(unsigned exploration_level,
        unsigned nb_threads,
        const Timeout& timeout,
        const std::vector<HeuristicParameters>& h_param) const = 0;
};

} // namespace vroom

#endif
