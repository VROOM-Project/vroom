#ifndef CVRP_OR_OPT_H
#define CVRP_OR_OPT_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/ls_operator.h"

class cvrp_or_opt : public ls_operator {
protected:
  gain_t normal_stored_gain;
  gain_t reversed_stored_gain;

  bool reverse_s_edge;

  virtual void compute_gain() override;

public:
  cvrp_or_opt(const input& input,
              const solution_state& sol_state,
              raw_route& s_route,
              index_t s_vehicle,
              index_t s_rank,
              raw_route& t_route,
              index_t t_vehicle,
              index_t t_rank);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<index_t> addition_candidates() const override;

  virtual std::vector<index_t> update_candidates() const override;
};

#endif
