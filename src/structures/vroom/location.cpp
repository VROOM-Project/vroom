/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "location.h"

location_t::location_t(index_t index) : index(index) {
}

location_t::location_t(index_t index, coordinate_t lon, coordinate_t lat)
  : index(index), lon(lon), lat(lat) {
}

bool location_t::has_coordinates() const {
  return (lon != boost::none) and (lat != boost::none);
}
