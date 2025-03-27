#ifndef SOLUTION_STATE_H
#define SOLUTION_STATE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/bbox.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/tw_route.h"

namespace vroom::utils {

class SolutionState {
private:
  const Input& _input;
  const std::size_t _nb_vehicles;

public:
  // Store unassigned jobs.
  std::unordered_set<Index> unassigned;

  // fwd_costs[v][new_v][i] stores the total cost from job at rank 0
  // to job at rank i in the route for vehicle v, from the point of
  // view of a vehicle new_v. bwd_costs[v][new_v][i] stores the total
  // cost from job at rank i to job at rank 0 (i.e. when *reversing*
  // all edges) in the route for vehicle v, from the point of view of
  // a vehicle new_v.
  std::vector<std::vector<std::vector<Eval>>> fwd_costs;
  std::vector<std::vector<std::vector<Eval>>> bwd_costs;

  // fwd_skill_rank[v1][v2] stores the maximum rank r for a step in
  // route for vehicle v1 such that v2 can handle all jobs from step 0
  // to r -- excluded -- in that route. bwd_skill_rank[v1][v2] stores
  // the minimum rank r for a step in route for vehicle v1 such that
  // v2 can handle all jobs after step r -- included -- up to the end
  // of that route.
  std::vector<std::vector<Index>> fwd_skill_rank;
  std::vector<std::vector<Index>> bwd_skill_rank;

  // fwd_priority[v][i] stores the sum of priorities from job at rank
  // 0 to job at rank i (included) in the route for vehicle v.
  // bwd_priority[v][i] stores the sum of priorities from job at rank
  // i to last job in the route for vehicle v.
  std::vector<std::vector<Priority>> fwd_priority;
  std::vector<std::vector<Priority>> bwd_priority;

  // edge_evals_around_node[v][i] evaluates the sum of edges that
  // appear before and after job at rank i in route for vehicle v
  // (handling cases where those edges are absent or linked with
  // start/end of vehicle). node_gains[v][i] stores potential gain
  // when removing job at rank i in route for vehicle
  // v. node_candidates[v] is the rank that yields the biggest such
  // gain for vehicle v.
  std::vector<std::vector<Eval>> edge_evals_around_node;
  std::vector<std::vector<Eval>> node_gains;
  std::vector<Index> node_candidates;

  // edge_evals_around_edge[v][i] evaluates the sum of edges that
  // appear before and after edge starting at rank i in route for
  // vehicle v (handling cases where those edges are absent or linked
  // with start/end of vehicle). edge_gains[v][i] stores potential
  // gain when removing edge starting at rank i in route for vehicle
  // v. edge_candidates[v] is the rank that yields the biggest such
  // gain for vehicle v.
  std::vector<std::vector<Eval>> edge_evals_around_edge;
  std::vector<std::vector<Eval>> edge_gains;
  std::vector<Index> edge_candidates;

  // pd_gains[v][i] stores potential gain when removing pickup at rank
  // i in route for vehicle v along with it's associated delivery.
  std::vector<std::vector<Eval>> pd_gains;

  // If job at rank i in route for vehicle v is a pickup
  // (resp. delivery), then matching_delivery_rank[v][i]
  // (resp. matching_pickup_rank[v][i]) stores the rank of the
  // matching delivery (resp. pickup).
  std::vector<std::vector<Index>> matching_delivery_rank;
  std::vector<std::vector<Index>> matching_pickup_rank;

  // cheapest_job_rank_in_routes_from[v1][v2][r1] stores the rank of
  // job in route v2 that minimize cost (as seen from the v2
  // perspective) from job at rank r1 in v1.
  std::vector<std::vector<std::vector<Index>>> cheapest_job_rank_in_routes_from;
  // cheapest_job_rank_in_routes_to[v1][v2][r1] stores the rank of job
  // in route v2 that minimize cost (as seen from the v2 perspective)
  // to job at rank r1 in v1.
  std::vector<std::vector<std::vector<Index>>> cheapest_job_rank_in_routes_to;

  // insertion_ranks_begin[v][j] is the highest rank in route for
  // vehicle v such that inserting job at rank j strictly before
  // insertion_ranks_begin[v][j] is bound to fail based on job
  // constraints and earliest/latest dates in route.
  // insertion_ranks_end[v][j] is the lowest rank in route for vehicle
  // v such that inserting job at rank j at insertion_ranks_end[v][j]
  // or after is bound to fail based on job constraints and
  // earliest/latest dates in route.
  std::vector<std::vector<Index>> insertion_ranks_begin;
  std::vector<std::vector<Index>> insertion_ranks_end;

  // weak_insertion_ranks_begin[v][j] is the highest rank in route for
  // vehicle v such that inserting job at rank j strictly before
  // weak_insertion_ranks_begin[v][j] is bound to fail based on job
  // constraints and route tasks time windows.
  // weak_insertion_ranks_end[v][j] is the lowest rank in route for
  // vehicle v such that inserting job at rank j at
  // weak_insertion_ranks_end[v][j] or after is bound to fail based on
  // job constraints and route tasks time windows. The range
  // restriction is weaker than right above but has the advantage of
  // remaining valid for use in operators that modify route for
  // vehicle v.
  std::vector<std::vector<Index>> weak_insertion_ranks_begin;
  std::vector<std::vector<Index>> weak_insertion_ranks_end;

  // Store evaluation of all routes, including fixed and travel costs.
  std::vector<Eval> route_evals;

  // Store bbox for all routes tasks (not including vehicle start and
  // end).
  std::vector<BBox> route_bbox;

  explicit SolutionState(const Input& input);

  template <class Route> void setup(const Route& r, Index v);

  template <class Solution> void setup(const Solution& sol);

  void update_costs(const std::vector<Index>& route, Index v);

  void update_skills(const std::vector<Index>& route, Index v1);

  void update_priorities(const std::vector<Index>& route, Index v);

  void set_node_gains(const std::vector<Index>& route, Index v);

  void set_edge_gains(const std::vector<Index>& route, Index v);

  void set_pd_gains(const std::vector<Index>& route, Index v);

  void set_pd_matching_ranks(const std::vector<Index>& route, Index v);

  void update_cheapest_job_rank_in_routes(const std::vector<Index>& route_1,
                                          const std::vector<Index>& route_2,
                                          Index v1,
                                          Index v2);

  void set_insertion_ranks(const RawRoute& r, Index v);
  void set_insertion_ranks(const TWRoute& r, Index v);

  void update_route_eval(const std::vector<Index>& route, Index v);

  void update_route_bbox(const std::vector<Index>& route, Index v);
};

} // namespace vroom::utils

#endif
