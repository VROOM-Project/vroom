#ifndef CVRP_INNER_RELOCATE_H
#define CVRP_INNER_RELOCATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/ls_operator.h"

class cvrp_inner_relocate : public ls_operator {
protected:
  virtual void compute_gain() override;

public:
  cvrp_inner_relocate(const input& input,
                      const solution_state& sol_state,
                      std::vector<index_t>& s_route,
                      index_t s_vehicle,
                      index_t s_rank,
                      index_t t_rank); // relocate rank *after* removal.

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<index_t> addition_candidates() const override;

  virtual std::vector<index_t> update_candidates() const override;
};

#endif
