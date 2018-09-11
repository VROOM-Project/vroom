#ifndef CVRP_LOCAL_SEARCH_H
#define CVRP_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution_state.h"

struct solution_indicators {
  unsigned unassigned;
  cost_t cost;
  unsigned used_vehicles;
};

class cvrp_local_search {
private:
  const input& _input;
  const matrix<cost_t>& _m;
  const std::size_t V;
  const amount_t _amount_lower_bound;
  const amount_t _double_amount_lower_bound;
  const unsigned _max_nb_jobs_removal;

  std::vector<index_t> _all_routes;
  raw_solution& _target_sol;
  raw_solution _sol;
  solution_state _sol_state;

  raw_solution _best_sol;
  unsigned _best_unassigned;
  cost_t _best_cost;

  void try_job_additions(const std::vector<index_t>& routes,
                         double regret_coeff);

  void run_tsp(index_t route_rank);

  void run_ls_step();

  void remove_from_routes();

public:
  cvrp_local_search(const input& input,
                    raw_solution& sol,
                    unsigned max_nb_jobs_removal);

  solution_indicators indicators() const;

  void run();
};

#endif
