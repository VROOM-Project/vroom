#ifndef CVRP_MIXED_EXCHANGE_H
#define CVRP_MIXED_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace cvrp {

class MixedExchange : public ls::Operator {
private:
  bool _gain_upper_bound_computed;
  Eval _normal_s_gain;
  Eval _reversed_s_gain;

protected:
  bool reverse_t_edge;
  const bool check_t_reverse;

  bool s_is_normal_valid;
  bool s_is_reverse_valid;
  const Amount source_delivery;
  const Amount target_delivery;

  virtual void compute_gain() override;

public:
  MixedExchange(const Input& input,
                const utils::SolutionState& sol_state,
                RawRoute& s_route,
                Index s_vehicle,
                Index s_rank,
                RawRoute& t_route,
                Index t_vehicle,
                Index t_rank,
                bool check_t_reverse);

  // Compute and store all possible cost depending on whether edges
  // are reversed or not. Return only an upper bound for gain as
  // precise gain requires validity information.
  Eval gain_upper_bound();

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
