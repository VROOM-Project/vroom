#ifndef VRPTW_INNER_EXCHANGE_H
#define VRPTW_INNER_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/inner_exchange.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

class vrptw_inner_exchange : public cvrp_inner_exchange {
private:
  tw_solution& _tw_sol;

public:
  vrptw_inner_exchange(const input& input,
                       const solution_state& sol_state,
                       tw_solution& tw_sol,
                       index_t s_vehicle,
                       index_t s_rank,
                       index_t t_rank);

  virtual bool is_valid() const override;

  virtual void apply() const override;
};

#endif
