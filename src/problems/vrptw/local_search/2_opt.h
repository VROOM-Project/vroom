#ifndef VRPTW_TWO_OPT_H
#define VRPTW_TWO_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/2_opt.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

class vrptw_two_opt : public cvrp_two_opt {
private:
  tw_solution& _tw_sol;

public:
  vrptw_two_opt(const input& input,
                const solution_state& sol_state,
                tw_solution& tw_sol,
                index_t s_vehicle,
                index_t s_rank,
                index_t t_vehicle,
                index_t t_rank);

  virtual bool is_valid() override;

  virtual void apply() override;
};

#endif
