/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./solution_state.h"

solution_state::solution_state(std::size_t n)
  : fwd_amounts(n),
    bwd_amounts(n),
    fwd_costs(n),
    bwd_costs(n),
    fwd_skill_rank(n, std::vector<index_t>(n)),
    bwd_skill_rank(n, std::vector<index_t>(n)),
    edge_costs_around_node(n),
    node_gains(n),
    node_candidates(n),
    edge_costs_around_edge(n),
    edge_gains(n),
    edge_candidates(n),
    nearest_job_rank_in_routes_from(n, std::vector<std::vector<index_t>>(n)),
    nearest_job_rank_in_routes_to(n, std::vector<std::vector<index_t>>(n)) {
}
