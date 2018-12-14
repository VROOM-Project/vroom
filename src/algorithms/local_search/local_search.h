#ifndef VRPTW_LOCAL_SEARCH_H
#define VRPTW_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/raw_route.h"
#include "structures/vroom/solution_state.h"
#include "structures/vroom/tw_route.h"

template <class Route,
          class exchange,
          class cross_exchange,
          class mixed_exchange,
          class two_opt,
          class reverse_two_opt,
          class relocate,
          class or_opt,
          class intra_exchange,
          class intra_cross_exchange,
          class intra_mixed_exchange,
          class intra_relocate,
          class intra_or_opt>
class local_search {
private:
  const input& _input;
  const matrix<cost_t>& _m;
  const std::size_t V;
  const amount_t _amount_lower_bound;
  const amount_t _double_amount_lower_bound;

  const unsigned _max_nb_jobs_removal;
  std::vector<index_t> _all_routes;

  solution_state _sol_state;

  std::vector<Route> _sol;

  std::vector<Route>& _best_sol;
  unsigned _best_unassigned;
  cost_t _best_cost;

  void try_job_additions(const std::vector<index_t>& routes,
                         double regret_coeff);

  void run_ls_step();

  void remove_from_routes();

public:
  local_search(const input& input,
               std::vector<Route>& tw_sol,
               unsigned max_nb_jobs_removal);

  solution_indicators indicators() const;

  void run();
};

#endif
