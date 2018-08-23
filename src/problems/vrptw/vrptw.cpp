/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "problems/vrptw/vrptw.h"
#include "problems/vrptw/heuristics/best_insertion.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/tw_route.h"
#include "utils/helpers.h"

using tw_solution = std::vector<tw_route>;

vrptw::vrptw(const input& input) : vrp(input) {
}

solution vrptw::solve(unsigned exploration_level, unsigned nb_threads) const {
  tw_solution sol = best_insertion(_input);

  raw_solution raw_sol;
  raw_sol.reserve(sol.size());

  std::transform(sol.begin(),
                 sol.end(),
                 std::back_inserter(raw_sol),
                 [](const auto& tw_r) { return tw_r.route; });

  return format_solution(_input, raw_sol);
}
