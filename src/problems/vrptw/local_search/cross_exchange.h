#ifndef VRPTW_CROSS_EXCHANGE_H
#define VRPTW_CROSS_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/cross_exchange.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

class vrptw_cross_exchange : public cvrp_cross_exchange {
private:
  tw_solution& _tw_sol;

  bool _s_is_normal_valid;
  bool _s_is_reverse_valid;
  bool _t_is_normal_valid;
  bool _t_is_reverse_valid;

public:
  vrptw_cross_exchange(const input& input,
                       const solution_state& sol_state,
                       tw_solution& tw_sol,
                       index_t s_vehicle,
                       index_t s_rank,
                       index_t t_vehicle,
                       index_t t_rank);

  virtual gain_t gain() override;

  virtual bool is_valid();

  virtual void apply() const override;
};

#endif
