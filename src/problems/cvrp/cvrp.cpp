/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "cvrp.h"
#include "../../structures/vroom/input/input.h"

cvrp::cvrp(const input& input) : vrp(input) {
  for (const auto& v : _input._vehicles) {
    if (!v.has_capacity()) {
      throw custom_exception("Missing capacity for vehicle " +
                             std::to_string(v.id));
    }
  }
  for (const auto& j : _input._jobs) {
    if (!j.has_amount()) {
      throw custom_exception("Missing amount for job " + std::to_string(j.id));
    }
  }
}

bool cvrp::empty_cluster(const std::vector<index_t>& cluster, index_t v) const {
  // Checking if the cluster has only start/end.
  return (cluster.size() == 1) or
         ((cluster.size() == 2) and _input._vehicles[v].has_start() and
          _input._vehicles[v].has_end() and
          (_input._vehicles[v].start.get().index() !=
           _input._vehicles[v].end.get().index()));
}

solution cvrp::solve(unsigned nb_threads) const {
  std::vector<solution> tsp_sols;

  struct param {
    CLUSTERING_T type;
    INIT_T init;
    double regret_coeff;
  };

  std::vector<param> parameters;
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 0});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.5});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NONE, 1});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 0});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 0.5});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 1});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::HIGHER_AMOUNT, 0});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::HIGHER_AMOUNT, 0.5});
  parameters.push_back({CLUSTERING_T::PARALLEL, INIT_T::HIGHER_AMOUNT, 1});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 0});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 0.5});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 1});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 0});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 0.5});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 1});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 0});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 0.5});
  parameters.push_back({CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 1});

  std::vector<clustering> clusterings;
  for (const auto& p : parameters) {
    clusterings.emplace_back(_input, p.type, p.init, p.regret_coeff);
  }

  auto c =
    std::min_element(clusterings.begin(),
                     clusterings.end(),
                     [](auto& lhs, auto& rhs) {
                       return lhs.unassigned.size() < rhs.unassigned.size() or
                              (lhs.unassigned.size() ==
                                 rhs.unassigned.size() and
                               lhs.edges_cost < rhs.edges_cost);
                     });

  std::string strategy =
    (c->type == CLUSTERING_T::PARALLEL) ? "parallel" : "sequential";
  std::string init_str;
  switch (c->init) {
  case INIT_T::NONE:
    init_str = "none";
    break;
  case INIT_T::HIGHER_AMOUNT:
    init_str = "higher_amount";
    break;
  case INIT_T::NEAREST:
    init_str = "nearest";
    break;
  }
  std::cout << "Best clustering:" << strategy << ";" << init_str << ";"
            << c->regret_coeff << ";" << c->unassigned.size() << ";"
            << c->edges_cost << std::endl;

  for (std::size_t i = 0; i < c->clusters.size(); ++i) {
    if (empty_cluster(c->clusters[i], i)) {
      std::cout << "Empty cluster" << std::endl;
      continue;
    }

    for (const auto& j : c->clusters[i]) {
      std::cout << j << " ; ";
    }
    std::cout << std::endl;

    tsp p(_input, c->clusters[i], i);

    tsp_sols.push_back(p.solve(1));
  }

  std::vector<route_t> routes;
  cost_t total_cost = 0;
  for (const auto& tsp_sol : tsp_sols) {
    routes.push_back(tsp_sol.routes[0]);
    total_cost += tsp_sol.summary.cost;
  }

  return solution(0, total_cost, std::move(routes), std::move(c->unassigned));
}
