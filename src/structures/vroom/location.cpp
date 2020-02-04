/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/location.h"

namespace vroom {

Location::Location(Index index)
  : _index(index), _coords(boost::none), _user_index(true) {
}

Location::Location(Index index, const Coordinates& coords)
  : _index(index), _coords(OptionalCoordinates(coords)), _user_index(true) {
}

Location::Location(const Coordinates& coords)
  : _coords(OptionalCoordinates(coords)), _user_index(false) {
}

void Location::set_index(Index index) {
  assert(!_user_index);
  _index = index;
}

bool Location::has_coordinates() const {
  return _coords != boost::none;
}

Coordinate Location::lon() const {
  assert(this->has_coordinates());
  return _coords.get()[0];
}

Coordinate Location::lat() const {
  assert(this->has_coordinates());
  return _coords.get()[1];
}

bool Location::user_index() const {
  return _user_index;
}

bool Location::operator==(const Location& other) const {
  return (_user_index and (_index == other.index())) or
         (this->has_coordinates() and other.has_coordinates() and
          (this->lon() == other.lon()) and (this->lat() == other.lat()));
}

} // namespace vroom
