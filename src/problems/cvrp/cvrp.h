#ifndef CVRP_H
#define CVRP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrp.h"

namespace vroom {

class CVRP : public VRP {
private:
  static const std::vector<HeuristicParameters> homogeneous_parameters;
  static const std::vector<HeuristicParameters> heterogeneous_parameters;

public:
  explicit CVRP(const Input& input);

  Solution solve(unsigned nb_searches,
                 unsigned depth,
                 unsigned nb_threads,
                 const Timeout& timeout) const override;
};

} // namespace vroom

#endif
