/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/location.h"

namespace vroom {

Location::Location(Index index)
  : _index(index), _coords(std::nullopt), _user_index(true) {
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
  return _coords.has_value();
}

Coordinates Location::coordinates() const {
  assert(this->has_coordinates());
  return _coords.value();
}

Coordinate Location::lon() const {
  assert(this->has_coordinates());
  return _coords.value().lon;
}

Coordinate Location::lat() const {
  assert(this->has_coordinates());
  return _coords.value().lat;
}

bool Location::user_index() const {
  return _user_index;
}

bool Location::operator==(const Location& other) const {
  return (this->user_index() && other.user_index() &&
          (this->index() == other.index())) ||
         (this->has_coordinates() && other.has_coordinates() &&
          (this->lon() == other.lon()) && (this->lat() == other.lat()));
}

} // namespace vroom
