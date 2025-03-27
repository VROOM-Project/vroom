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
  explicit Location(Index index);

  Location(Index index, const Coordinates& coords);

  explicit Location(const Coordinates& coords);

  void set_index(Index index);

  bool has_coordinates() const;

  Index index() const {
    return _index;
  }

  Coordinates coordinates() const;

  Coordinate lon() const;

  Coordinate lat() const;

  bool user_index() const;

  // Locations are considered identical if they have the same
  // user-provided index or if they both have coordinates and those
  // are equal. The last part is required for situations with no
  // explicit index provided in input.
  bool operator==(const Location& other) const;
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
