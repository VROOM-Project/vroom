#ifndef VRPTW_INTRA_OR_OPT_H
#define VRPTW_INTRA_OR_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/operators/intra_or_opt.h"

namespace vroom::vrptw {

class IntraOrOpt : public cvrp::IntraOrOpt {
private:
  TWRoute& _tw_s_route;

public:
  IntraOrOpt(const Input& input,
             const utils::SolutionState& sol_state,
             TWRoute& tw_s_route,
             Index s_vehicle,
             Index s_rank,
             Index t_rank, // rank *after* removal.
             bool check_reverse);

  bool is_valid() override;

  void apply() override;

  std::vector<Index> addition_candidates() const override;
};

} // namespace vroom::vrptw

#endif
