/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/vrptw.h"
#include "algorithms/heuristics/heuristics.h"
#include "algorithms/local_search/local_search.h"
#include "problems/vrptw/operators/cross_exchange.h"
#include "problems/vrptw/operators/intra_cross_exchange.h"
#include "problems/vrptw/operators/intra_exchange.h"
#include "problems/vrptw/operators/intra_mixed_exchange.h"
#include "problems/vrptw/operators/intra_or_opt.h"
#include "problems/vrptw/operators/intra_relocate.h"
#include "problems/vrptw/operators/intra_two_opt.h"
#include "problems/vrptw/operators/mixed_exchange.h"
#include "problems/vrptw/operators/or_opt.h"
#include "problems/vrptw/operators/pd_shift.h"
#include "problems/vrptw/operators/priority_replace.h"
#include "problems/vrptw/operators/relocate.h"
#include "problems/vrptw/operators/reverse_two_opt.h"
#include "problems/vrptw/operators/route_exchange.h"
#include "problems/vrptw/operators/route_split.h"
#include "problems/vrptw/operators/swap_star.h"
#include "problems/vrptw/operators/tsp_fix.h"
#include "problems/vrptw/operators/two_opt.h"
#include "problems/vrptw/operators/unassigned_exchange.h"
#include "utils/helpers.h"

namespace vroom {

namespace vrptw {

using LocalSearch = ls::LocalSearch<TWRoute,
                                    vrptw::UnassignedExchange,
                                    vrptw::CrossExchange,
                                    vrptw::MixedExchange,
                                    vrptw::TwoOpt,
                                    vrptw::ReverseTwoOpt,
                                    vrptw::Relocate,
                                    vrptw::OrOpt,
                                    vrptw::IntraExchange,
                                    vrptw::IntraCrossExchange,
                                    vrptw::IntraMixedExchange,
                                    vrptw::IntraRelocate,
                                    vrptw::IntraOrOpt,
                                    vrptw::IntraTwoOpt,
                                    vrptw::PDShift,
                                    vrptw::RouteExchange,
                                    vrptw::SwapStar,
                                    vrptw::RouteSplit,
                                    vrptw::PriorityReplace,
                                    vrptw::TSPFix>;
} // namespace vrptw

const std::vector<HeuristicParameters> VRPTW::homogeneous_parameters =
  {HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.3),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.4),
   HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.3),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.4),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.5),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.4),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.5),

   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.6),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.7),

   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.7),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1.4),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.1),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.3),
   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.8),

   HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0.5),
   HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0.8),
   HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 2.4),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 1.2),

   HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 1),
   HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1.3),
   HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0),
   HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0.3),

   HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 2),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.9),
   HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 1)};

const std::vector<HeuristicParameters> VRPTW::heterogeneous_parameters =
  {HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.5),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.9),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.4),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.4),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.8),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.6),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.9),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.6),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 1.8),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 1.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 1.4),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.7),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 1.3),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 2.4),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.3),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 1.2),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 1.2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.6),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 1.6),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.2),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 1.7),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.5),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 1.5),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 1.5),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 2.2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 2.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.5),

   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 1.2),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.1),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.9),
   HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 1.1)};

VRPTW::VRPTW(const Input& input) : VRP(input) {
}

Solution VRPTW::solve(unsigned nb_searches,
                      unsigned depth,
                      unsigned nb_threads,
                      const Timeout& timeout,
                      const std::vector<HeuristicParameters>& h_param) const {
  return VRP::solve<TWRoute, vrptw::LocalSearch>(nb_searches,
                                                 depth,
                                                 nb_threads,
                                                 timeout,
                                                 h_param,
                                                 homogeneous_parameters,
                                                 heterogeneous_parameters);
}

} // namespace vroom
