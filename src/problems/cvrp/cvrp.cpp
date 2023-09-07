/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/cvrp.h"
#include "algorithms/heuristics/heuristics.h"
#include "algorithms/local_search/local_search.h"
#include "problems/cvrp/operators/cross_exchange.h"
#include "problems/cvrp/operators/intra_cross_exchange.h"
#include "problems/cvrp/operators/intra_exchange.h"
#include "problems/cvrp/operators/intra_mixed_exchange.h"
#include "problems/cvrp/operators/intra_or_opt.h"
#include "problems/cvrp/operators/intra_relocate.h"
#include "problems/cvrp/operators/intra_two_opt.h"
#include "problems/cvrp/operators/mixed_exchange.h"
#include "problems/cvrp/operators/or_opt.h"
#include "problems/cvrp/operators/pd_shift.h"
#include "problems/cvrp/operators/relocate.h"
#include "problems/cvrp/operators/reverse_two_opt.h"
#include "problems/cvrp/operators/route_exchange.h"
#include "problems/cvrp/operators/route_split.h"
#include "problems/cvrp/operators/swap_star.h"
#include "problems/cvrp/operators/two_opt.h"
#include "problems/cvrp/operators/unassigned_exchange.h"
#include "problems/tsp/tsp.h"
#include "utils/helpers.h"

namespace vroom {

namespace cvrp {

using LocalSearch = ls::LocalSearch<RawRoute,
                                    cvrp::UnassignedExchange,
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
                                    cvrp::IntraTwoOpt,
                                    cvrp::PDShift,
                                    cvrp::RouteExchange,
                                    cvrp::SwapStar,
                                    cvrp::RouteSplit>;
} // namespace cvrp

const std::vector<HeuristicParameters> CVRP::homogeneous_parameters =
  {HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.3),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 0.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 0.5),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.3),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.7),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 0.3),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 2.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.2),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.6),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.9),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 2.4),

   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 0.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 0.8),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 1.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.4),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 1.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 1.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 2.4),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 0),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 0.9),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 1.5),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.5),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 1.3),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 1.5),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.1),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.4),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 1.6),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHEST_AMOUNT, 0.6),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.5)};

const std::vector<HeuristicParameters> CVRP::heterogeneous_parameters =
  {HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.4),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 0.2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.3),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.3),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.6),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 0.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.2),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.9),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 0.3),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.4),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.5),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 1.4),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 2.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 0),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 0.8),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.6),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.8),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 0.4),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 0.5),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 0.6),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 0.7),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 1.2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.5),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.7),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHEST_AMOUNT, 1),
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
      _input.vehicles[0].steps.empty() and
      !_input.vehicles[0].has_max_travel_time()) {
    // This is a plain TSP, no need to go through the trouble below.
    std::vector<Index> job_ranks(_input.jobs.size());
    std::iota(job_ranks.begin(), job_ranks.end(), 0);

    TSP p(_input, std::move(job_ranks), 0);

    RawRoute r(_input, 0, 0);
    r.set_route(_input, p.raw_solve(nb_threads, timeout));

    return utils::format_solution(_input, {r});
  }

  return VRP::solve<RawRoute, cvrp::LocalSearch>(exploration_level,
                                                 nb_threads,
                                                 timeout,
                                                 h_param,
                                                 homogeneous_parameters,
                                                 heterogeneous_parameters);
}

} // namespace vroom
