#ifndef LOCATION_H
#define LOCATION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <cassert>

#include "structures/typedefs.h"

namespace vroom {

class Location {
private:
  // Index of this location in the matrix.
  Index _index;
  // Coordinates (not mandatory).
  OptionalCoordinates _coords;
  bool _user_index;

public:
  explicit Location(Index index)
    : _index(index), _coords(std::nullopt), _user_index(true) {
  }

  Location(Index index, const Coordinates& coords)
    : _index(index), _coords(OptionalCoordinates(coords)), _user_index(true) {
  }

  explicit Location(const Coordinates& coords)
    : _coords(OptionalCoordinates(coords)), _user_index(false) {
  }

  void set_index(Index index) {
    assert(!_user_index);
    _index = index;
  }

  bool has_coordinates() const {
    return _coords.has_value();
  }

  Index index() const {
    return _index;
  }

  Coordinates coordinates() const {
    assert(this->has_coordinates());
    return _coords.value();
  }

  Coordinate lon() const {
    assert(this->has_coordinates());
    return _coords.value().lon;
  }

  Coordinate lat() const {
    assert(this->has_coordinates());
    return _coords.value().lat;
  }

  bool user_index() const {
    return _user_index;
  }

  // Locations are considered identical if they have the same
  // user-provided index or if they both have coordinates and those
  // are equal. The last part is required for situations with no
  // explicit index provided in input.
  bool operator==(const Location& other) const {
    return (this->user_index() && other.user_index() &&
            (this->index() == other.index())) ||
           (this->has_coordinates() && other.has_coordinates() &&
            (this->lon() == other.lon()) && (this->lat() == other.lat()));
  }
};

} // namespace vroom

namespace std {
template <> struct hash<vroom::Location> {
  std::size_t operator()(const vroom::Location& l) const noexcept {
    if (l.user_index()) {
      return hash<vroom::Index>()(l.index());
    }

    assert(l.has_coordinates());
    return ((hash<vroom::Coordinate>()(l.lon()) ^
             (hash<vroom::Coordinate>()(l.lat()) << 1)) >>
            1);
  }
};
} // namespace std

#endif
