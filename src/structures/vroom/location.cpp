/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/location.h"

location_t::location_t(index_t index)
  : _index(index), _coords(boost::none), _user_index(true) {
}

location_t::location_t(index_t index, const coords_t& coords)
  : _index(index), _coords(optional_coords_t(coords)), _user_index(true) {
}

location_t::location_t(const coords_t& coords)
  : _coords(optional_coords_t(coords)), _user_index(false) {
}

void location_t::set_index(index_t index) {
  assert(!_user_index);
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

bool location_t::user_index() const {
  return _user_index;
}

bool location_t::operator==(const location_t& other) const {
  return (_index == other.index()) or
         (this->has_coordinates() and other.has_coordinates() and
          (this->lon() == other.lon()) and (this->lat() == other.lat()));
}
