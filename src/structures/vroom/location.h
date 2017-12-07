#ifndef LOCATION_H
#define LOCATION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../typedefs.h"

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
};

#endif
