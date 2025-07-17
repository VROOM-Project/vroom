/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "algorithms/local_search/top_insertions.h"
#include "utils/helpers.h"

namespace vroom::ls {

void update_insertions(ThreeInsertions& insertions, InsertionOption&& option) {
  if (option.eval < insertions[2].eval) {
    if (option.eval < insertions[1].eval) {
      if (option.eval < insertions[0].eval) {
        insertions[2] = std::move(insertions[1]);
        insertions[1] = std::move(insertions[0]);
        insertions[0] = std::move(option);
      } else {
        insertions[2] = std::move(insertions[1]);
        insertions[1] = std::move(option);
      }
    } else {
      insertions[2] = std::move(option);
    }
  }
}

template <class Route>
ThreeInsertions find_top_3_insertions(const Input& input,
                                      Index j,
                                      const Route& r) {
  const auto& v = input.vehicles[r.v_rank];

  auto best_insertions = empty_three_insertions;

  for (Index rank = 0; rank <= r.route.size(); ++rank) {
    InsertionOption current_insert =
      {utils::addition_eval(input, j, v, r.route, rank), rank};

    update_insertions(best_insertions, std::move(current_insert));
  }

  return best_insertions;
}

template ThreeInsertions find_top_3_insertions(const Input& input,
                                               Index j,
                                               const RawRoute& r);

template ThreeInsertions find_top_3_insertions(const Input& input,
                                               Index j,
                                               const TWRoute& r);

} // namespace vroom::ls
