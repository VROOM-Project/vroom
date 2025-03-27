#ifndef CVRP_INTRA_OR_OPT_H
#define CVRP_INTRA_OR_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::cvrp {

class IntraOrOpt : public ls::Operator {
private:
  bool _gain_upper_bound_computed{false};
  Eval _normal_t_gain;
  Eval _reversed_t_gain{NO_GAIN};

protected:
  bool reverse_s_edge{false};

  bool is_normal_valid{false};
  bool is_reverse_valid{false};
  const bool check_reverse;

  std::vector<Index> _moved_jobs;
  const Index _first_rank;
  const Index _last_rank;
  const Amount _delivery;
  Index _s_edge_first;
  Index _s_edge_last;

  void compute_gain() override;

public:
  IntraOrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             RawRoute& s_raw_route,
             Index s_vehicle,
             Index s_rank,
             Index t_rank, // rank *after* removal.
             bool check_reverse);

  // Compute and store all possible cost depending on whether edges
  // are reversed or not. Return only an upper bound for gain as
  // precise gain requires validity information.
  Eval gain_upper_bound();

  bool is_valid() override;

  void apply() override;

  std::vector<Index> addition_candidates() const override;

  std::vector<Index> update_candidates() const override;
};

} // namespace vroom::cvrp

#endif
