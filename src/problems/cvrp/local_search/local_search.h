#ifndef CVRP_LOCAL_SEARCH_H
#define CVRP_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/local_search.h"

class cvrp_local_search : public local_search {
private:
  const unsigned _max_nb_jobs_removal;

  std::vector<index_t> _all_routes;
  raw_solution& _target_sol;
  raw_solution _sol;

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

  virtual solution_indicators indicators() const override;

  virtual void run() override;
};

#endif
