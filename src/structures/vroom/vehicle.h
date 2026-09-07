#ifndef VEHICLE_H
#define VEHICLE_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

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
  const Cost per_km;
  const Cost per_task_hour;

  explicit VehicleCosts(UserCost fixed = 0,
                        UserCost per_hour = DEFAULT_COST_PER_HOUR,
                        UserCost per_km = DEFAULT_COST_PER_KM,
                        UserCost per_task_hour = DEFAULT_COST_PER_TASK_HOUR)
    : fixed(utils::scale_from_user_cost(fixed)),
      per_hour(static_cast<Cost>(per_hour)),
      per_km(static_cast<Cost>(per_km)),
      per_task_hour(static_cast<Cost>(per_task_hour)){};

  friend bool operator==(const VehicleCosts& lhs,
                         const VehicleCosts& rhs) = default;

  friend bool operator<(const VehicleCosts& lhs, const VehicleCosts& rhs) {
    return std::tie(lhs.fixed, lhs.per_hour, lhs.per_km, lhs.per_task_hour) <
           std::tie(rhs.fixed, rhs.per_hour, rhs.per_km, rhs.per_task_hour);
  }
};

struct Vehicle {
  const Id id;
  std::optional<Location> start;
  std::optional<Location> end;
  const std::string profile;
  // Alternative capacity vectors: a load is valid iff it fits at
  // least one of them (component-wise). Empty when the vehicle has a
  // single capacity vector, in which case capacity alone applies.
  const std::vector<Amount> capacities;
  // Component-wise maximum over all capacity vectors (the "hull"),
  // or the single capacity vector. Fitting the hull is a necessary
  // condition for a valid load, so this can be used as a relaxation.
  const Amount capacity;
  const Skills skills;
  const TimeWindow tw;
  const std::vector<Break> breaks;
  const std::string description;
  const VehicleCosts costs;
  CostWrapper cost_wrapper;
  size_t max_tasks;
  const Duration max_travel_time;
  const Distance max_distance;
  const bool has_break_max_load;
  std::vector<VehicleStep> steps;
  Index type;
  const std::string type_str;
  std::unordered_map<Id, Index> break_id_to_rank;
  // Ids of the vehicle groups this vehicle belongs to (see
  // VehicleGroup): at most max_vehicles of a group can be used.
  const std::vector<Id> groups;

  Vehicle(
    Id id,
    const std::optional<Location>& start,
    const std::optional<Location>& end,
    std::string profile = DEFAULT_PROFILE,
    const Amount& capacity = Amount(0),
    Skills skills = Skills(),
    const TimeWindow& tw = TimeWindow(),
    const std::vector<Break>& breaks = std::vector<Break>(),
    std::string description = "",
    const VehicleCosts& costs = VehicleCosts(),
    double speed_factor = 1.,
    const std::optional<size_t>& max_tasks = std::optional<size_t>(),
    const std::optional<UserDuration>& max_travel_time =
      std::optional<UserDuration>(),
    const std::optional<UserDistance>& max_distance =
      std::optional<UserDistance>(),
    const std::vector<VehicleStep>& input_steps = std::vector<VehicleStep>(),
    std::string type_str = NO_TYPE,
    const std::vector<Amount>& capacities = std::vector<Amount>(),
    std::vector<Id> groups = std::vector<Id>());

  // Whether validity of a load requires checking against several
  // capacity vectors (when false, load <= capacity is exact).
  bool has_alternative_capacities() const {
    return !capacities.empty();
  }

  // Whether the vehicle can carry a given load at some point in its
  // route, i.e. the load fits at least one of the capacity vectors.
  template <typename E> bool can_carry(const AmountExpression<E>& load) const {
    if (capacities.empty()) {
      return load <= capacity;
    }
    return std::ranges::any_of(capacities, [&load](const auto& c) {
      return load <= c;
    });
  }

  bool has_start() const;

  bool has_end() const;

  bool has_same_locations(const Vehicle& other) const;

  bool has_same_profile(const Vehicle& other) const;

  bool cost_based_on_metrics() const;

  Duration available_duration() const;

  Cost fixed_cost() const {
    return costs.fixed;
  }

  Cost task_cost(Duration task_duration) const {
    return costs.per_task_hour * task_duration;
  }

  Eval task_eval(Duration task_duration) const {
    return Eval(task_cost(task_duration), 0, 0, task_duration);
  }

  Duration duration(Index i, Index j) const {
    return cost_wrapper.duration(i, j);
  }

  Cost cost(Index i, Index j) const {
    return cost_wrapper.cost(i, j);
  }

  Eval eval(Index i, Index j) const {
    return Eval(cost_wrapper.cost(i, j),
                cost_wrapper.duration(i, j),
                cost_wrapper.distance(i, j));
  }

  bool ok_for_travel_time(Duration d) const {
    assert(0 <= d);
    return d <= max_travel_time;
  }

  bool ok_for_distance(Distance d) const {
    assert(0 <= d);
    return d <= max_distance;
  }

  bool ok_for_range_bounds(const Eval& e) const {
    assert(0 <= e.duration && 0 <= e.distance);
    return e.duration <= max_travel_time && e.distance <= max_distance;
  }

  bool has_range_bounds() const;

  Index break_rank(Id break_id) const;

  friend bool operator<(const Vehicle& lhs, const Vehicle& rhs) {
    // Sort by:
    //   - decreasing max_tasks
    //   - decreasing capacity
    //   - decreasing TW length
    //   - decreasing range (max travel time and distance)
    return std::tie(rhs.max_tasks,
                    rhs.capacity,
                    rhs.tw.length,
                    rhs.max_travel_time,
                    rhs.max_distance) < std::tie(lhs.max_tasks,
                                                 lhs.capacity,
                                                 lhs.tw.length,
                                                 lhs.max_travel_time,
                                                 lhs.max_distance);
  }
};

} // namespace vroom

#endif
