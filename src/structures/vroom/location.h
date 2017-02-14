#ifndef LOCATION_H
#define LOCATION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../typedefs.h"

struct location_t{
  // Index of this location in the matrix.
  const index_t index;

  // Coordinates (not mandatory).
  boost::optional<const coordinate_t> lon;
  boost::optional<const coordinate_t> lat;

  location_t(index_t index);

  location_t(index_t index, coordinate_t lon, coordinate_t lat);

  bool has_coordinates() const;
};

#endif
