#ifndef CVRP_INNER_OR_OPT_H
#define CVRP_INNER_OR_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/ls_operator.h"

class cvrp_inner_or_opt : public ls_operator {
protected:
  virtual void compute_gain() override;

  gain_t normal_stored_gain;
  gain_t reversed_stored_gain;

  bool reverse_s_edge;

public:
  cvrp_inner_or_opt(const input& input,
                    const solution_state& sol_state,
                    std::vector<index_t>& s_route,
                    index_t s_vehicle,
                    index_t s_rank,
                    index_t t_rank); // rank *after* removal.

  virtual bool is_valid() const override;

  virtual void apply() const override;

  virtual std::vector<index_t> addition_candidates() const override;

  virtual std::vector<index_t> update_candidates() const override;
};

#endif
