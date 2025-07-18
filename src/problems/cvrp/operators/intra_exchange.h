#ifndef CVRP_INTRA_EXCHANGE_H
#define CVRP_INTRA_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::cvrp {

class IntraExchange : public ls::Operator {
protected:
  std::vector<Index> _moved_jobs;
  const Index _first_rank;
  const Index _last_rank;
  const Amount _delivery;

  void prepare_moved_jobs();
  void compute_gain() override;

public:
  IntraExchange(const Input& input,
                const utils::SolutionState& sol_state,
                RawRoute& s_raw_route,
                Index s_vehicle,
                Index s_rank,
                Index t_rank);

  bool is_valid() override;

  void apply() override;

  std::vector<Index> addition_candidates() const override;

  std::vector<Index> update_candidates() const override;
};

} // namespace vroom::cvrp

#endif
