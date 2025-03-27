#ifndef CVRP_REVERSE_TWO_OPT_H
#define CVRP_REVERSE_TWO_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::cvrp {

class ReverseTwoOpt : public ls::Operator {
protected:
  void compute_gain() override;

  const Amount _s_delivery;
  const Amount _t_delivery;

public:
  ReverseTwoOpt(const Input& input,
                const utils::SolutionState& sol_state,
                RawRoute& s_route,
                Index s_vehicle,
                Index s_rank,
                RawRoute& t_route,
                Index t_vehicle,
                Index t_rank);

  bool is_valid() override;

  void apply() override;

  std::vector<Index> addition_candidates() const override;

  std::vector<Index> update_candidates() const override;
};

} // namespace vroom::cvrp

#endif
