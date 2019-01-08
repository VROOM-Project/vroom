#ifndef CVRP_H
#define CVRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>

#include "problems/vrp.h"

namespace vroom {

class CVRP : public VRP {
private:
  bool empty_cluster(const std::vector<Index>& cluster, Index v) const;

  static constexpr std::array<HeuristicParameters, 32> homogeneous_parameters =
    {HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.3),
     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.7),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.1),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.2),

     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 2.1),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.3),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::HIGHER_AMOUNT, 0.2),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NEAREST, 0),

     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.9),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 2.4),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::FURTHEST, 0.2),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::FURTHEST, 0.6),

     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.4),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.8),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::HIGHER_AMOUNT, 0.3),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NEAREST, 2.4),

     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.2),
     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 1.2),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1.2),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.4),

     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NONE, 0.9),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NONE, 1.5),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::HIGHER_AMOUNT, 0.7),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NEAREST, 1.7),

     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.5),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 1.6),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NONE, 0.5),

     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NONE, 0.7),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NONE, 1),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NEAREST, 0.4),
     HeuristicParameters(CLUSTERING::SEQUENTIAL, INIT::NEAREST, 1.8)};

  static constexpr std::array<HeuristicParameters, 32>
    heterogeneous_parameters =
      {HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.2),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.1),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.3),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NONE, 0.3),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.4),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.6),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NONE, 0.9),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NEAREST, 0.6),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.3),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.1),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.2),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NEAREST, 0),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.3),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.4),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NONE, 0.5),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NONE, 1.9),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.2),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.9),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.6),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NONE, 0.4),

       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NEAREST, 0.1),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NEAREST, 0.5),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NEAREST, 0.9),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NEAREST, 1.8),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 0.1),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.8),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NONE, 0),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NONE, 0.8),

       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NONE, 1),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NEAREST, 0.3),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NEAREST, 1.2),
       HeuristicParameters(CLUSTERING::PARALLEL, INIT::NEAREST, 2.3)};

public:
  CVRP(const Input& input);

  virtual Solution solve(unsigned exploration_level,
                         unsigned nb_threads) const override;
};

} // namespace vroom

#endif
