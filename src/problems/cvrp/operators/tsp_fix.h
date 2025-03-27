#ifndef CVRP_TSP_FIX_H
#define CVRP_TSP_FIX_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"

namespace vroom::cvrp {

class TSPFix : public ls::Operator {
protected:
  std::vector<Index> tsp_route;

  void compute_gain() override;

  const Amount _s_delivery;

public:
  TSPFix(const Input& input,
         const utils::SolutionState& sol_state,
         RawRoute& s_route,
         Index s_vehicle);

  bool is_valid() override;

  void apply() override;

  std::vector<Index> addition_candidates() const override;

  std::vector<Index> update_candidates() const override;
};

} // namespace vroom::cvrp

#endif
