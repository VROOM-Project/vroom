#ifndef CVRP_LOCAL_SEARCH_H
#define CVRP_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../structures/typedefs.h"
#include "../../../utils/output_json.h" // To remove

class cvrp_local_search {
private:
  const input& _input;
  const matrix<cost_t>& _m;
  const std::size_t V;
  raw_solution& _sol;
  std::vector<amount_t> _amounts;
  // _nearest_job_rank_in_routes_from[v1][v2][r1] stores the rank of
  // job in route v2 that minimize cost from job at rank r1 in v1.
  std::vector<std::vector<std::vector<index_t>>>
    _nearest_job_rank_in_routes_from;
  // _nearest_job_rank_in_routes_to[v1][v2][r1] stores the rank of job
  // in route v2 that minimize cost to job at rank r1 in v1.
  std::vector<std::vector<std::vector<index_t>>> _nearest_job_rank_in_routes_to;

  void update_nearest_job_rank_in_routes(index_t v1, index_t v2);

  void set_node_gains(index_t v);
  void set_edge_gains(index_t v);

public:
  cvrp_local_search(const input& input, raw_solution& sol);

  void run();
};

#endif
