#ifndef VRPTW_INNER_MIXED_EXCHANGE_H
#define VRPTW_INNER_MIXED_EXCHANGE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/cvrp/local_search/intra_mixed_exchange.h"
#include "structures/vroom/tw_route.h"

class vrptw_intra_mixed_exchange : public cvrp_intra_mixed_exchange {
private:
  tw_route& _tw_s_route;

  bool _s_is_normal_valid;
  bool _s_is_reverse_valid;

  std::vector<index_t> _moved_jobs;
  const index_t _first_rank;
  const index_t _last_rank;
  index_t _t_edge_first;
  index_t _t_edge_last;

  virtual void compute_gain() override;

public:
  vrptw_intra_mixed_exchange(const input& input,
                             const solution_state& sol_state,
                             tw_route& tw_s_route,
                             index_t s_vehicle,
                             index_t s_rank,
                             index_t t_rank);

  virtual bool is_valid() override;

  virtual void apply() override;

  virtual std::vector<index_t> addition_candidates() const override;
};

#endif
