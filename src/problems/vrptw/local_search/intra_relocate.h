#ifndef VRPTW_INNER_RELOCATE_H
#define VRPTW_INNER_RELOCATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/intra_relocate.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

class vrptw_intra_relocate : public cvrp_intra_relocate {
private:
  tw_solution& _tw_sol;

  std::vector<index_t> _moved_jobs;
  const index_t _first_rank;
  const index_t _last_rank;

public:
  vrptw_intra_relocate(const input& input,
                       const solution_state& sol_state,
                       tw_solution& tw_sol,
                       index_t s_vehicle,
                       index_t s_rank,
                       index_t t_rank); // relocate rank *after* removal.

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<index_t> addition_candidates() const override;
};

#endif
