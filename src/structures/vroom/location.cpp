/*

This file is part of VROOM.

Copyright (c) 2015-2017, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "location.h"

location_t::location_t(index_t index)
  : _index(index), _coords(boost::none), user_index(true) {
}

location_t::location_t(index_t index, const coords_t& coords)
  : _index(index), _coords(optional_coords_t(coords)), user_index(true) {
}

location_t::location_t(const coords_t& coords)
  : _coords(optional_coords_t(coords)), user_index(false) {
}

void location_t::set_index(index_t index) {
  assert(!user_index);
  _index = index;
}

bool location_t::has_coordinates() const {
  return _coords != boost::none;
}

index_t location_t::index() const {
  return _index;
}

coordinate_t location_t::lon() const {
  assert(this->has_coordinates());
  return _coords.get()[0];
}

coordinate_t location_t::lat() const {
  assert(this->has_coordinates());
  return _coords.get()[1];
}
