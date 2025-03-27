/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/bbox.h"

namespace vroom {

void BBox::extend(Coordinates c) {
  min.lon = std::min(min.lon, c.lon);
  min.lat = std::min(min.lat, c.lat);

  max.lon = std::max(max.lon, c.lon);
  max.lat = std::max(max.lat, c.lat);
}

bool BBox::intersects(const BBox& other) const {
  return other.min.lon <= max.lon && other.min.lat <= max.lat &&
         min.lon <= other.max.lon && min.lat <= other.max.lat;
}

} // namespace vroom
