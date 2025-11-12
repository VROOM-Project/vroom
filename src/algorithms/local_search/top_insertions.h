#ifndef TOP_INSERTIONS_H
#define TOP_INSERTIONS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/input/input.h"

namespace vroom::ls {

struct InsertionOption {
  Eval eval;
  Index rank;
};

using ThreeInsertions = std::array<InsertionOption, 3>;

constexpr InsertionOption no_insert = {NO_EVAL, 0};
constexpr ThreeInsertions
  empty_three_insertions({no_insert, no_insert, no_insert});

template <class Route>
ThreeInsertions find_top_3_insertions(const Input& input,
                                      Index j,
                                      const Route& r);

} // namespace vroom::ls

#endif
