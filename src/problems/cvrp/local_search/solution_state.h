#ifndef SOLUTION_STATE_H
#define SOLUTION_STATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../../../structures/typedefs.h"
#include "../../../structures/vroom/amount.h"

struct solution_state {
  // fwd_amounts[v][i] stores the total amount up to rank i in the
  // route for vehicle v, while bwd_amounts[v][i] stores the total
  // amount *after* rank i in the route for vehicle v.
  std::vector<std::vector<amount_t>> fwd_amounts;
  std::vector<std::vector<amount_t>> bwd_amounts;

  // fwd_costs[v][i] stores the total cost from job at rank 0 to job
  // at rank i in the route for vehicle v, while bwd_costs[v][i]
  // stores the total cost from job at rank i to job at rank 0
  // (i.e. when *reversing* all edges).
  std::vector<std::vector<cost_t>> fwd_costs;
  std::vector<std::vector<cost_t>> bwd_costs;

  // fwd_skill_rank[v1][v2] stores the maximum rank r for a step in
  // route for vehicle v1 such that v2 can handle all jobs from step 0
  // to r -- excluded -- in that route. bwd_skill_rank[v1][v2] stores
  // the minimum rank r for a step in route for vehicle v1 such that
  // v2 can handle all jobs after step r -- included -- up to the end
  // of that route.
  std::vector<std::vector<index_t>> fwd_skill_rank;
  std::vector<std::vector<index_t>> bwd_skill_rank;

  // edge_costs_around_node[v][i] stores the sum of costs for edges
  // that appear before and after job at rank i in route for vehicle v
  // (handling cases where those edges are absent or linked with
  // start/end of vehicle). node_gains[v][i] stores potential gain
  // when removing job at rank i in route for vehicle
  // v. node_candidates[v] is the rank that yields the biggest such
  // gain for vehicle v.
  std::vector<std::vector<gain_t>> edge_costs_around_node;
  std::vector<std::vector<gain_t>> node_gains;
  std::vector<index_t> node_candidates;

  // edge_costs_around_edge[v][i] stores the sum of costs for edges
  // that appear before and after edge starting at rank i in route for
  // vehicle v (handling cases where those edges are absent or linked
  // with start/end of vehicle). edge_gains[v][i] stores potential
  // gain when removing edge starting at rank i in route for vehicle
  // v. edge_candidates[v] is the rank that yields the biggest such
  // gain for vehicle v.
  std::vector<std::vector<gain_t>> edge_costs_around_edge;
  std::vector<std::vector<gain_t>> edge_gains;
  std::vector<index_t> edge_candidates;

  // nearest_job_rank_in_routes_from[v1][v2][r1] stores the rank of
  // job in route v2 that minimize cost from job at rank r1 in v1.
  std::vector<std::vector<std::vector<index_t>>>
    nearest_job_rank_in_routes_from;
  // nearest_job_rank_in_routes_to[v1][v2][r1] stores the rank of job
  // in route v2 that minimize cost to job at rank r1 in v1.
  std::vector<std::vector<std::vector<index_t>>> nearest_job_rank_in_routes_to;

  solution_state(std::size_t n);
};

#endif
