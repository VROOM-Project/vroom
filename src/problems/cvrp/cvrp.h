#ifndef CVRP_H
#define CVRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>

#include "problems/cvrp/heuristics/clustering.h"
#include "problems/vrp.h"

class cvrp : public vrp {
private:
  bool empty_cluster(const std::vector<index_t>& cluster, index_t v) const;

  static constexpr std::array<h_param, 32> homogeneous_parameters =
    {h_param(HEURISTIC_T::BASIC, INIT_T::NONE, 0.3),
     h_param(HEURISTIC_T::BASIC, INIT_T::NONE, 0.7),
     h_param(HEURISTIC_T::BASIC, INIT_T::HIGHER_AMOUNT, 0.1),
     h_param(HEURISTIC_T::BASIC, INIT_T::FURTHEST, 0.2),

     h_param(HEURISTIC_T::BASIC, INIT_T::HIGHER_AMOUNT, 2.1),
     h_param(HEURISTIC_T::BASIC, INIT_T::FURTHEST, 0.3),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 0.2),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 0),

     h_param(HEURISTIC_T::BASIC, INIT_T::HIGHER_AMOUNT, 0.9),
     h_param(HEURISTIC_T::BASIC, INIT_T::HIGHER_AMOUNT, 2.4),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::FURTHEST, 0.2),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::FURTHEST, 0.6),

     h_param(HEURISTIC_T::BASIC, INIT_T::NONE, 0.4),
     h_param(HEURISTIC_T::BASIC, INIT_T::HIGHER_AMOUNT, 0.8),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 0.3),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 2.4),

     h_param(HEURISTIC_T::BASIC, INIT_T::NONE, 0.2),
     h_param(HEURISTIC_T::BASIC, INIT_T::NONE, 1.2),
     h_param(HEURISTIC_T::BASIC, INIT_T::HIGHER_AMOUNT, 1.2),
     h_param(HEURISTIC_T::BASIC, INIT_T::FURTHEST, 0.4),

     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 0.9),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 1.5),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::HIGHER_AMOUNT, 0.7),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 1.7),

     h_param(HEURISTIC_T::BASIC, INIT_T::NONE, 0.5),
     h_param(HEURISTIC_T::BASIC, INIT_T::HIGHER_AMOUNT, 1),
     h_param(HEURISTIC_T::BASIC, INIT_T::FURTHEST, 1.6),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 0.5),

     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 0.7),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NONE, 1),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 0.4),
     h_param(CLUSTERING_T::SEQUENTIAL, INIT_T::NEAREST, 1.8)};

  static constexpr std::array<h_param, 32> heterogeneous_parameters =
    {h_param(HEURISTIC_T::DYNAMIC, INIT_T::HIGHER_AMOUNT, 0.2),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::FURTHEST, 0.1),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::FURTHEST, 0.2),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.3),

     h_param(HEURISTIC_T::DYNAMIC, INIT_T::NONE, 0.2),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::NONE, 0.5),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::HIGHER_AMOUNT, 0.1),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::FURTHEST, 0.3),

     h_param(HEURISTIC_T::DYNAMIC, INIT_T::NONE, 0.3),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::NONE, 0.4),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::HIGHER_AMOUNT, 0.6),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 0.6),

     h_param(HEURISTIC_T::DYNAMIC, INIT_T::NONE, 0.7),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::HIGHER_AMOUNT, 0.3),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.5),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 1.2),

     h_param(HEURISTIC_T::DYNAMIC, INIT_T::NONE, 0.6),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::HIGHER_AMOUNT, 0.9),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::FURTHEST, 0.4),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.2),

     h_param(HEURISTIC_T::DYNAMIC, INIT_T::FURTHEST, 0.6),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NONE, 0),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.7),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 1.9),

     h_param(HEURISTIC_T::DYNAMIC, INIT_T::NONE, 0.1),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::NONE, 0.8),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::NONE, 1),
     h_param(HEURISTIC_T::DYNAMIC, INIT_T::FURTHEST, 0.7),

     h_param(CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.8),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NONE, 0.9),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NONE, 1.5),
     h_param(CLUSTERING_T::PARALLEL, INIT_T::NEAREST, 0.4)};

public:
  cvrp(const input& input);

  virtual solution solve(unsigned exploration_level,
                         unsigned nb_threads) const override;
};

#endif
