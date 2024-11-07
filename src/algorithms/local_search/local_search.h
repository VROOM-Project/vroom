#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution_indicators.h"
#include "structures/vroom/solution_state.h"

#ifdef LOG_LS
#include "algorithms/local_search/log_local_search.h"
#endif

namespace vroom::ls {

template <class Route,
          class UnassignedExchange,
          class CrossExchange,
          class MixedExchange,
          class TwoOpt,
          class ReverseTwoOpt,
          class Relocate,
          class OrOpt,
          class IntraExchange,
          class IntraCrossExchange,
          class IntraMixedExchange,
          class IntraRelocate,
          class IntraOrOpt,
          class IntraTwoOpt,
          class PDShift,
          class RouteExchange,
          class SwapStar,
          class RouteSplit,
          class PriorityReplace,
          class TSPFix>
class LocalSearch {
private:
  const Input& _input;
  const std::size_t _nb_vehicles;

  const unsigned _depth;
  const Deadline _deadline;

  std::optional<unsigned> _completed_depth;
  std::vector<Index> _all_routes;

  utils::SolutionState _sol_state;

  std::vector<Route> _sol;

  std::vector<Route>& _best_sol;
  utils::SolutionIndicators _best_sol_indicators;

#ifdef LOG_LS_OPERATORS
  // Store operator usage stats.
  std::array<unsigned, OperatorName::MAX> tried_moves;
  std::array<unsigned, OperatorName::MAX> applied_moves;
#endif

#ifdef LOG_LS
  std::vector<log::Step> steps;
#endif

  void recreate(const std::vector<Index>& routes
#ifdef LOG_LS
                ,
                bool log_addition_step = false
#endif
  );

  void run_ls_step();

  // Compute "cost" between route at rank v_target and job with rank r
  // in route at rank v. Relies on
  // _sol_state.cheapest_job_rank_in_routes_* being up to date.
  Eval job_route_cost(Index v_target, Index v, Index r);

  // Compute lower bound for the cost of relocating job at rank r
  // (resp. jobs at rank r1 and r2) in route v to any other
  // (compatible) route.
  Eval relocate_cost_lower_bound(Index v, Index r);
  Eval relocate_cost_lower_bound(Index v, Index r1, Index r2);

  void remove_from_routes();

public:
  LocalSearch(const Input& input,
              std::vector<Route>& tw_sol,
              unsigned depth,
              const Timeout& timeout);

  utils::SolutionIndicators indicators() const;

  void run();

#ifdef LOG_LS_OPERATORS
  std::array<OperatorStats, OperatorName::MAX> get_stats() const;
#endif

#ifdef LOG_LS
  std::vector<log::Step> get_steps() const {
    return steps;
  }
#endif
};

} // namespace vroom::ls

#endif
