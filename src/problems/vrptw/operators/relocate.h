#ifndef VRPTW_RELOCATE_H
#define VRPTW_RELOCATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/relocate.h"
#include "structures/vroom/tw_route.h"

class vrptw_relocate : public cvrp_relocate {
private:
  tw_route& _tw_s_route;
  tw_route& _tw_t_route;

public:
  vrptw_relocate(const input& input,
                 const solution_state& sol_state,
                 tw_route& tw_s_route,
                 index_t s_vehicle,
                 index_t s_rank,
                 tw_route& tw_t_route,
                 index_t t_vehicle,
                 index_t t_rank);

  virtual bool is_valid() override;

  virtual void apply() override;
};

#endif
