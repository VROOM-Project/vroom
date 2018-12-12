#ifndef CVRP_INNER_MIXED_EXCHANGE_H
#define CVRP_INNER_MIXED_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/ls_operator.h"

class cvrp_intra_mixed_exchange : public ls_operator {
protected:
  gain_t normal_s_gain;
  gain_t reversed_s_gain;
  gain_t t_gain;

  bool reverse_t_edge;

  virtual void compute_gain() override;

public:
  cvrp_intra_mixed_exchange(const input& input,
                            const solution_state& sol_state,
                            raw_route& s_route,
                            index_t s_vehicle,
                            index_t s_rank,
                            index_t t_rank);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<index_t> addition_candidates() const override;

  virtual std::vector<index_t> update_candidates() const override;
};

#endif
