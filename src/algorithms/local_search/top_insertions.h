#ifndef TOP_INSERTIONS_H
#define TOP_INSERTIONS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "utils/helpers.h"

namespace vroom::ls {

struct InsertionOption {
  Eval cost;
  Index rank;
};

using ThreeInsertions = std::array<InsertionOption, 3>;

constexpr InsertionOption NO_INSERT = {NO_EVAL, 0};
constexpr ThreeInsertions
  EMPTY_THREE_INSERTIONS({NO_INSERT, NO_INSERT, NO_INSERT});

inline ThreeInsertions find_top_3_insertions(const Input& input,
                                             Index j,
                                             Index v,
                                             const std::vector<Index>& route) {
  const auto& vehicle = input.vehicles[v];

  auto best_insertions = EMPTY_THREE_INSERTIONS;

  for (Index rank = 0; rank <= route.size(); ++rank) {
    const InsertionOption current_insert =
      {utils::addition_cost(input, j, vehicle, route, rank), rank};

    if (current_insert.cost < best_insertions[2].cost) {
      if (current_insert.cost < best_insertions[1].cost) {
        if (current_insert.cost < best_insertions[0].cost) {
          best_insertions[2] = best_insertions[1];
          best_insertions[1] = best_insertions[0];
          best_insertions[0] = current_insert;
        } else {
          best_insertions[2] = best_insertions[1];
          best_insertions[1] = current_insert;
        }
      } else {
        best_insertions[2] = current_insert;
      }
    }
  }

  return best_insertions;
}

} // namespace vroom::ls

#endif
