#ifndef CVRP_ROUTE_SPLIT_H
#define CVRP_ROUTE_SPLIT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"
#include "algorithms/local_search/route_split_utils.h"

namespace vroom::cvrp {

class RouteSplit : public ls::Operator {
protected:
  const Eval _best_known_gain;
  const std::vector<Index> _empty_route_ranks;
  const std::vector<std::reference_wrapper<RawRoute>> _empty_route_refs;
  ls::SplitChoice choice;

  static std::vector<std::reference_wrapper<RawRoute>> dummy_route_refs;

  virtual void compute_gain() override;

public:
  RouteSplit(const Input& input,
             const utils::SolutionState& sol_state,
             RawRoute& s_route,
             Index s_vehicle,
             std::vector<Index> empty_route_ranks,
             std::vector<std::reference_wrapper<RawRoute>> empty_route_refs,
             const Eval& best_known_gain);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;

  virtual bool invalidated_by(Index rank) const override;
};

} // namespace vroom::cvrp

#endif
