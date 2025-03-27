#ifndef CVRP_CROSS_EXCHANGE_H
#define CVRP_CROSS_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::cvrp {

class CrossExchange : public ls::Operator {
private:
  bool _gain_upper_bound_computed{false};
  Eval _normal_s_gain;
  Eval _reversed_s_gain{NO_GAIN};
  Eval _normal_t_gain;
  Eval _reversed_t_gain{NO_GAIN};

protected:
  bool reverse_s_edge{false};
  bool reverse_t_edge{false};
  const bool check_s_reverse;
  const bool check_t_reverse;

  bool s_is_normal_valid{false};
  bool s_is_reverse_valid{false};
  bool t_is_normal_valid{false};
  bool t_is_reverse_valid{false};

  const Amount source_delivery;
  const Amount target_delivery;

  void compute_gain() override;

public:
  CrossExchange(const Input& input,
                const utils::SolutionState& sol_state,
                RawRoute& s_route,
                Index s_vehicle,
                Index s_rank,
                RawRoute& t_route,
                Index t_vehicle,
                Index t_rank,
                bool check_s_reverse,
                bool check_t_reverse);

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
