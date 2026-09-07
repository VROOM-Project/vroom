#ifndef VEHICLE_GROUP_H
#define VEHICLE_GROUP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <vector>

#include "structures/typedefs.h"

namespace vroom {

// A set of vehicles of which at most max_vehicles may be used (have
// a non-empty route) in a solution. Typical use: several alternative
// configurations of the same physical vehicle listed as distinct
// vehicles, or a pool of drivers smaller than the fleet.
struct VehicleGroup {
  const Id id;
  const unsigned max_vehicles;
  const std::string description;
  // Ranks (in Input::vehicles) of the member vehicles, filled by
  // Input::add_vehicle.
  std::vector<Index> vehicles;

  VehicleGroup(Id id, unsigned max_vehicles, std::string description = "")
    : id(id), max_vehicles(max_vehicles), description(std::move(description)) {
  }
};

} // namespace vroom

#endif
