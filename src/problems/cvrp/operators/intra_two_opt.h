#ifndef CVRP_INTRA_TWO_OPT_H
#define CVRP_INTRA_TWO_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::cvrp {

class IntraTwoOpt : public ls::Operator {
protected:
  void compute_gain() override;

  const Amount delivery;

public:
  IntraTwoOpt(const Input& input,
              const utils::SolutionState& sol_state,
              RawRoute& s_route,
              Index s_vehicle,
              Index s_rank,
              Index t_rank);

  bool reversal_ok_for_shipments() const;

  bool is_valid() override;

  void apply() override;

  std::vector<Index> addition_candidates() const override;

  std::vector<Index> update_candidates() const override;
};

} // namespace vroom::cvrp

#endif
