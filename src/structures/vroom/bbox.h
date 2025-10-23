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
  void extend(Coordinates c) {
    min.lon = std::min(min.lon, c.lon);
    min.lat = std::min(min.lat, c.lat);

    max.lon = std::max(max.lon, c.lon);
    max.lat = std::max(max.lat, c.lat);
  }

  bool intersects(const BBox& other) const {
    return other.min.lon <= max.lon && other.min.lat <= max.lat &&
           min.lon <= other.max.lon && min.lat <= other.max.lat;
  }
};

} // namespace vroom

#endif
