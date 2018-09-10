#ifndef LOCATION_H
#define LOCATION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

class location_t {
private:
  // Index of this location in the matrix.
  index_t _index;
  // Coordinates (not mandatory).
  optional_coords_t _coords;
  bool _user_index;

public:
  location_t(index_t index);

  location_t(index_t index, const coords_t& coords);

  location_t(const coords_t& coords);

  void set_index(index_t index);

  bool has_coordinates() const;

  index_t index() const;

  coordinate_t lon() const;

  coordinate_t lat() const;

  bool user_index() const;

  // Locations are considered identical if they have the same
  // user-provided index or if they both have coordinates and those
  // are equal. The last part is required for situations with no
  // explicit index provided in input.
  bool operator==(const location_t& other) const;
};

#endif
