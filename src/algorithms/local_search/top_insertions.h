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

constexpr InsertionOption no_insert = {NO_EVAL, 0};
constexpr ThreeInsertions
  empty_three_insertions({no_insert, no_insert, no_insert});

template <class Route>
inline ThreeInsertions find_top_3_insertions(const Input& input,
                                             Index j,
                                             const Route& r) {
  const auto& v = input.vehicles[r.vehicle_rank];

  auto best_insertions = empty_three_insertions;

  for (Index rank = 0; rank <= r.route.size(); ++rank) {
    const InsertionOption current_insert =
      {utils::addition_cost(input, j, v, r.route, rank), rank};

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
