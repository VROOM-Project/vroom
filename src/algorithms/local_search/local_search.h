#ifndef LOCAL_SEARCH_H
#define LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution_state.h"

namespace vroom {
namespace ls {

template <class Route,
          class UnassignedExchange,
          class SwapStar,
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
          class PDShift,
          class RouteExchange>
class LocalSearch {
private:
  const Input& _input;
  const std::size_t _nb_vehicles;

  const unsigned _max_nb_jobs_removal;
  const Deadline _deadline;

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

  void try_job_additions(const std::vector<Index>& routes, double regret_coeff);

  void run_ls_step();

  // Compute "cost" between route at rank v_target and job with rank r
  // in route at rank v. Relies on
  // _sol_state.cheapest_job_rank_in_routes_* being up to date.
  Gain job_route_cost(Index v_target, Index v, Index r);

  // Compute lower bound for the cost of relocating job at rank r
  // (resp. jobs at rank r1 and r2) in route v to any other
  // (compatible) route.
  Gain relocate_cost_lower_bound(Index v, Index r);
  Gain relocate_cost_lower_bound(Index v, Index r1, Index r2);

  void remove_from_routes();

public:
  LocalSearch(const Input& input,
              std::vector<Route>& tw_sol,
              unsigned max_nb_jobs_removal,
              const Timeout& timeout);

  utils::SolutionIndicators indicators() const;

  void run();

#ifdef LOG_LS_OPERATORS
  std::array<OperatorStats, OperatorName::MAX> get_stats() const;
#endif
};

} // namespace ls
} // namespace vroom

#endif
