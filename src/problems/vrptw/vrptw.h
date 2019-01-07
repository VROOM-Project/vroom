#ifndef VRPTW_H
#define VRPTW_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>

#include "problems/vrp.h"

namespace vroom {

class VRPTW : public VRP {
private:
  static constexpr std::array<HeuristicParameters, 32> homogeneous_parameters =
    {HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.3),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.4),
     HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0.2),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.3),

     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.4),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.5),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.4),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.5),

     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.1),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.6),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.2),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.7),

     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.2),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 0.7),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1.4),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.1),

     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0),
     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.1),
     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.3),
     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 0.8),

     HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0.5),
     HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0.8),
     HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 2.4),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 1.2),

     HeuristicParameters(HEURISTIC::BASIC, INIT::NONE, 1),
     HeuristicParameters(HEURISTIC::BASIC, INIT::HIGHER_AMOUNT, 1.3),
     HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0),
     HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 0.3),

     HeuristicParameters(HEURISTIC::BASIC, INIT::EARLIEST_DEADLINE, 2),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 0.9),
     HeuristicParameters(HEURISTIC::BASIC, INIT::FURTHEST, 1)};

  static constexpr std::array<HeuristicParameters, 32>
    heterogeneous_parameters =
      {HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.5),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.9),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.4),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.4),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.8),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.6),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.9),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.6),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 1.8),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 1.1),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 1.4),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.7),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 1.3),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 2.4),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.3),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 1.2),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 1.2),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 0.6),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 1.6),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.2),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 1.7),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 2),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.5),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 1.5),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 1.5),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::NONE, 2.2),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::HIGHER_AMOUNT, 2.1),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 0.5),

       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::EARLIEST_DEADLINE, 1.2),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.1),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 0.9),
       HeuristicParameters(HEURISTIC::DYNAMIC, INIT::FURTHEST, 1.1)};

public:
  VRPTW(const Input& input);

  virtual Solution solve(unsigned exploration_level,
                         unsigned nb_threads) const override;
};

} // namespace vroom

#endif
