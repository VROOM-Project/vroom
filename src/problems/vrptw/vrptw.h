#ifndef VRPTW_H
#define VRPTW_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrp.h"

namespace vroom {

class VRPTW : public VRP {
private:
  static const std::vector<HeuristicParameters> homogeneous_parameters;
  static const std::vector<HeuristicParameters> heterogeneous_parameters;

public:
  explicit VRPTW(const Input& input);

  Solution
  solve(unsigned nb_searches,
        unsigned depth,
        unsigned nb_threads,
        const Timeout& timeout,
        const std::vector<HeuristicParameters>& h_param) const override;
};

} // namespace vroom

#endif
