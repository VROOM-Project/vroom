#ifndef BBOX_H
#define BBOX_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

namespace vroom {

class BBox {

private:
  Coordinates min{std::numeric_limits<Coordinate>::max(),
                  std::numeric_limits<Coordinate>::max()};
  Coordinates max{std::numeric_limits<Coordinate>::min(),
                  std::numeric_limits<Coordinate>::min()};

public:
  void extend(Coordinates c);

  bool intersects(const BBox& other) const;
};

} // namespace vroom

#endif
