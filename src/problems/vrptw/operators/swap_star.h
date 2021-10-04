#ifndef VRPTW_SWAP_STAR_H
#define VRPTW_SWAP_STAR_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/swap_star.h"

namespace vroom {
namespace vrptw {

class SwapStar : public cvrp::SwapStar {
private:
  TWRoute& _tw_s_route;
  TWRoute& _tw_t_route;

  virtual void compute_gain() override;

public:
  SwapStar(const Input& input,
           const utils::SolutionState& sol_state,
           TWRoute& tw_s_route,
           Index s_vehicle,
           TWRoute& tw_t_route,
           Index t_vehicle,
           Gain best_known_gain);

  virtual void apply() override;
};

} // namespace vrptw
} // namespace vroom

#endif
