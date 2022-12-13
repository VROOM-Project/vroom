#ifndef VEHICLE_H
#define VEHICLE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <unordered_map>

#include "structures/typedefs.h"
#include "structures/vroom/amount.h"
#include "structures/vroom/break.h"
#include "structures/vroom/cost_wrapper.h"
#include "structures/vroom/eval.h"
#include "structures/vroom/input/vehicle_step.h"
#include "structures/vroom/location.h"
#include "structures/vroom/time_window.h"

namespace vroom {

struct VehicleCosts {
  const Cost fixed;
  const Cost per_hour;

  VehicleCosts(UserCost fixed = 0, UserCost per_hour = DEFAULT_COST_PER_HOUR)
    : fixed(COST_FACTOR * utils::scale_from_user_duration(fixed)),
      per_hour(static_cast<Cost>(per_hour)){};

  friend bool operator==(const VehicleCosts& lhs, const VehicleCosts& rhs) {
    return lhs.fixed == rhs.fixed and lhs.per_hour == rhs.per_hour;
  }

  friend bool operator<(const VehicleCosts& lhs, const VehicleCosts& rhs) {
    return lhs.fixed < rhs.fixed or
           (lhs.fixed == rhs.fixed and lhs.per_hour < rhs.per_hour);
  }
};

struct Vehicle {
  const Id id;
  std::optional<Location> start;
  std::optional<Location> end;
  const std::string profile;
  const Amount capacity;
  const Skills skills;
  const TimeWindow tw;
  const std::vector<Break> breaks;
  const std::string description;
  const VehicleCosts costs;
  CostWrapper cost_wrapper;
  size_t max_tasks;
  const Duration max_travel_time;
  const bool has_break_max_load;
  std::vector<VehicleStep> steps;
  std::unordered_map<Id, Index> break_id_to_rank;

  Vehicle(
    Id id,
    const std::optional<Location>& start,
    const std::optional<Location>& end,
    const std::string& profile = DEFAULT_PROFILE,
    const Amount& capacity = Amount(0),
    const Skills& skills = Skills(),
    const TimeWindow& tw = TimeWindow(),
    const std::vector<Break>& breaks = std::vector<Break>(),
    const std::string& description = "",
    const VehicleCosts& costs = VehicleCosts(),
    double speed_factor = 1.,
    const size_t max_tasks = std::numeric_limits<size_t>::max(),
    const std::optional<UserDuration>& max_travel_time =
      std::optional<UserDuration>(),
    const std::vector<VehicleStep>& input_steps = std::vector<VehicleStep>());

  bool has_start() const;

  bool has_end() const;

  bool has_same_locations(const Vehicle& other) const;

  bool has_same_profile(const Vehicle& other) const;

  bool cost_based_on_duration() const;

  Duration available_duration() const;

  Cost fixed_cost() const {
    return costs.fixed;
  }

  Duration duration(Index i, Index j) const {
    return cost_wrapper.duration(i, j);
  }

  Cost cost(Index i, Index j) const {
    return cost_wrapper.cost(i, j);
  }

  Eval eval(Index i, Index j) const {
    return Eval(cost_wrapper.cost(i, j), cost_wrapper.duration(i, j));
  }

  bool ok_for_travel_time(Duration d) const {
    assert(0 <= d);
    return d <= max_travel_time;
  }

  Index break_rank(Id id) const;
};

} // namespace vroom

#endif
