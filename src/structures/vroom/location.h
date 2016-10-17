#ifndef LOCATION_H
#define LOCATION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../typedefs.h"

struct location{
  // Index of this location in the matrix.
  const index_t index;

  // Coordinates (not mandatory).
  boost::optional<const coordinate_t> lon;
  boost::optional<const coordinate_t> lat;

  location(index_t index):
    index(index){}

  location(index_t index, coordinate_t lon, coordinate_t lat):
    index(index), lon(lon), lat(lat){}

  bool has_coordinates() const{
    return (lon != boost::none) and (lat != boost::none);
  }
};

#endif
