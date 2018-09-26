#ifndef VRPTW_LOCAL_SEARCH_H
#define VRPTW_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/local_search.h"
#include "structures/vroom/tw_route.h"

using tw_solution = std::vector<tw_route>;

class vrptw_local_search : public local_search {
private:
  tw_solution& _tw_sol;

  static unsigned ls_rank;

  // TODO remove
  bool log;
  unsigned log_iter;
  std::string log_name;
  void log_current_solution();

  void try_job_additions(const std::vector<index_t>& routes,
                         double regret_coeff);

  void straighten_route(index_t route_rank);

public:
  vrptw_local_search(const input& input, tw_solution& tw_sol);

  virtual solution_indicators indicators() const override;

  virtual void run() override;
};

#endif
