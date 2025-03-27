#ifndef MATRICES_H
#define MATRICES_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

namespace vroom::routing {

struct Matrices {
  Matrix<UserDuration> durations;
  Matrix<UserDistance> distances;

  explicit Matrices(std::size_t n) : durations(n), distances(n){};
};

} // namespace vroom::routing
#endif
