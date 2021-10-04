#ifndef CVRP_SWAP_STAR_H
#define CVRP_SWAP_STAR_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/operator.h"
#include "algorithms/local_search/swap_star_utils.h"

namespace vroom {
namespace cvrp {

class SwapStar : public ls::Operator {
protected:
  const Gain _best_known_gain;
  ls::SwapChoice choice;

  virtual void compute_gain() override;

public:
  SwapStar(const Input& input,
           const utils::SolutionState& sol_state,
           RawRoute& s_route,
           Index s_vehicle,
           RawRoute& t_route,
           Index t_vehicle,
           Gain best_known_gain);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<Index> addition_candidates() const override;

  virtual std::vector<Index> update_candidates() const override;
};

} // namespace cvrp
} // namespace vroom

#endif
