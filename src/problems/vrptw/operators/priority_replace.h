#ifndef VRPTW_PRIORITY_REPLACE_H
#define VRPTW_PRIORITY_REPLACE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/priority_replace.h"

namespace vroom::vrptw {

class PriorityReplace : public cvrp::PriorityReplace {
private:
  TWRoute& _tw_s_route;

public:
  PriorityReplace(const Input& input,
                  const utils::SolutionState& sol_state,
                  std::unordered_set<Index>& unassigned,
                  TWRoute& tw_s_route,
                  Index s_vehicle,
                  Index s_rank,
                  Index t_rank,
                  Index u,
                  Priority best_known_priority_gain);

  bool is_valid() override;

  void apply() override;
};

} // namespace vroom::vrptw

#endif
