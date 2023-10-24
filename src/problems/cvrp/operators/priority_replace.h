#ifndef CVRP_PRIORITY_REPLACE_H
#define CVRP_PRIORITY_REPLACE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::cvrp {

class PriorityReplace : public ls::Operator {
protected:
  const Index _u; // Unassigned job to insert.
  std::unordered_set<Index>& _unassigned;

  void compute_gain() override;

public:
  PriorityReplace(const Input& input,
                  const utils::SolutionState& sol_state,
                  std::unordered_set<Index>& unassigned,
                  RawRoute& s_raw_route,
                  Index s_vehicle,
                  Index s_rank,
                  Index u);

  bool is_valid() override;

  void apply() override;

  std::vector<Index> addition_candidates() const override;

  std::vector<Index> update_candidates() const override;

  std::vector<Index> required_unassigned() const override;
};

} // namespace vroom::cvrp

#endif
