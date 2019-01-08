#ifndef CVRP_INTRA_MIXED_EXCHANGE_H
#define CVRP_INTRA_MIXED_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom {
namespace cvrp {

class IntraMixedExchange : public ls::Operator {
protected:
  Gain normal_s_gain;
  Gain reversed_s_gain;
  Gain t_gain;

  bool reverse_t_edge;

  virtual void compute_gain() override;

public:
  IntraMixedExchange(const Input& input,
                     const utils::SolutionState& sol_state,
                     RawRoute& s_route,
                     Index s_vehicle,
                     Index s_rank,
                     Index t_rank);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
