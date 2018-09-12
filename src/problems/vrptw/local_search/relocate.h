#ifndef VRPTW_RELOCATE_H
#define VRPTW_RELOCATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/relocate.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

class vrptw_relocate : public cvrp_relocate {
private:
  tw_solution& _tw_sol;

public:
  vrptw_relocate(const input& input,
                 tw_solution& tw_sol,
                 raw_solution& sol,
                 const solution_state& sol_state,
                 index_t source_vehicle,
                 index_t source_rank,
                 index_t target_vehicle,
                 index_t target_rank);

  virtual bool is_valid() const override;

  virtual void apply() const override;
};

#endif
