/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "cvrp.h"
#include "../../structures/vroom/input/input.h"

cvrp::cvrp(const input& input) : vrp(input) {
}

solution cvrp::solve(unsigned nb_threads) const {
  std::vector<solution> tsp_sols;

  auto clusters = clustering(_input);
  // Test cutting the problem in several TSP (dumbly adding jobs in
  // order for now).
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    for (const auto& j : clusters[i]) {
      std::cout << j << " ; ";
    }
    std::cout << std::endl;

    tsp p(_input, clusters[i], i);

    tsp_sols.push_back(p.solve(1));
  }

  std::vector<route_t> routes;
  cost_t total_cost = 0;
  for (const auto& tsp_sol : tsp_sols) {
    routes.push_back(tsp_sol.routes[0]);
    total_cost += tsp_sol.summary.cost;
  }

  return solution(0, std::move(routes), total_cost);
}
