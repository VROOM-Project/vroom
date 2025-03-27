#ifndef CVRP_ROUTE_SPLIT_H
#define CVRP_ROUTE_SPLIT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"
#include "algorithms/local_search/route_split_utils.h"

namespace vroom::cvrp {

class RouteSplit : public ls::Operator {
protected:
  const Eval _best_known_gain;
  const std::vector<Index>& _empty_route_ranks;
  std::vector<RawRoute>& _sol;
  Index _begin_route_rank;
  Index _end_route_rank;
  ls::SplitChoice choice{ls::empty_route_split_choice};

  static std::vector<RawRoute> dummy_sol;

  void compute_gain() override;

public:
  RouteSplit(const Input& input,
             const utils::SolutionState& sol_state,
             RawRoute& s_route,
             Index s_vehicle,
             const std::vector<Index>& empty_route_ranks,
             std::vector<RawRoute>& sol,
             const Eval& best_known_gain);

  bool is_valid() override;

  void apply() override;

  std::vector<Index> addition_candidates() const override;

  std::vector<Index> update_candidates() const override;

  bool invalidated_by(Index rank) const override;
};

} // namespace vroom::cvrp

#endif
