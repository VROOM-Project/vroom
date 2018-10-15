#ifndef VRPTW_INNER_OR_OPT_H
#define VRPTW_INNER_OR_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/inner_or_opt.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

class vrptw_inner_or_opt : public cvrp_inner_or_opt {
private:
  tw_solution& _tw_sol;

  bool _is_normal_valid;
  bool _is_reverse_valid;

  virtual void compute_gain() override;

public:
  vrptw_inner_or_opt(const input& input,
                     const solution_state& sol_state,
                     tw_solution& tw_sol,
                     index_t s_vehicle,
                     index_t s_rank,
                     index_t t_rank); // rank *after* removal.

  virtual bool is_valid();

  virtual void apply() const override;
};

#endif
