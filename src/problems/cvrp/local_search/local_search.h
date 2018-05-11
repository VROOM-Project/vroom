#ifndef CVRP_LOCAL_SEARCH_H
#define CVRP_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../structures/typedefs.h"
#include "../../../utils/output_json.h" // To remove

class input;
class amount_t;

class cvrp_local_search {
private:
  const input& _input;
  raw_solution& _sol;
  std::vector<amount_t> _amounts;

  index_t previous_index(index_t route_rank, index_t job_rank) const;
  index_t next_index(index_t route_rank, index_t job_rank) const;
  void set_node_gains(index_t v);
  void set_edge_gains(index_t v);

public:
  cvrp_local_search(const input& input, raw_solution& sol);

  void run();
};

#endif
