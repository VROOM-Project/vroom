/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>

#include "structures/vroom/vehicle.h"
#include "utils/exception.h"

namespace vroom {

Vehicle::Vehicle(Id id,
                 const std::optional<Location>& start,
                 const std::optional<Location>& end,
                 std::string profile,
                 const Amount& capacity,
                 Skills skills,
                 const TimeWindow& tw,
                 const std::vector<Break>& breaks,
                 std::string description,
                 const VehicleCosts& costs,
                 double speed_factor,
                 const std::optional<size_t>& max_tasks,
                 const std::optional<UserDuration>& max_travel_time,
                 const std::optional<UserDistance>& max_distance,
                 const std::vector<VehicleStep>& input_steps,
                 std::string type_str)
  : id(id),
    start(start),
    end(end),
    profile(std::move(profile)),
    capacity(capacity),
    skills(std::move(skills)),
    tw(tw),
    breaks(breaks),
    description(std::move(description)),
    costs(costs),
    cost_wrapper(speed_factor, costs.per_hour, costs.per_km),
    max_tasks(max_tasks.value_or(DEFAULT_MAX_TASKS)),
    max_travel_time(max_travel_time.has_value()
                      ? utils::scale_from_user_duration(max_travel_time.value())
                      : DEFAULT_MAX_TRAVEL_TIME),
    max_distance(max_distance.has_value() ? max_distance.value()
                                          : DEFAULT_MAX_DISTANCE),
    has_break_max_load(std::ranges::any_of(breaks,
                                           [](const auto& b) {
                                             return b.max_load.has_value();
                                           })),
    type_str(std::move(type_str)) {
  if (!static_cast<bool>(start) && !static_cast<bool>(end)) {
    throw InputException(
      std::format("No start or end specified for vehicle {}.", id));
  }

  for (unsigned i = 0; i < breaks.size(); ++i) {
    const auto& b = breaks[i];

    if (break_id_to_rank.contains(b.id)) {
      throw InputException(std::format("Duplicate break id: {}.", b.id));
    }
    break_id_to_rank[b.id] = i;

    if (b.max_load.has_value() &&
        b.max_load.value().size() != capacity.size()) {
      throw InputException(
        std::format("Inconsistent break max_load size for break {}.", b.id));
    }
  }

  if (!input_steps.empty()) {
    // Populating steps. We rely on always having start and end steps
    // in input, so just add them if they're missing.
    using enum STEP_TYPE;

    steps.reserve(input_steps.size() + 2);

    unsigned rank_after_start = 0;
    if (input_steps.front().type == START) {
      steps.push_back(input_steps.front());
      rank_after_start = 1;
    } else {
      steps.emplace_back(START);
    }

    for (unsigned i = rank_after_start; i < input_steps.size(); ++i) {
      if (input_steps[i].type == START) {
        throw InputException(
          std::format("Unexpected start in input steps for vehicle {}.", id));
      }
      if (input_steps[i].type == END && (i != input_steps.size() - 1)) {
        throw InputException(
          std::format("Unexpected end in input steps for vehicle {}.", id));
      }

      steps.push_back(input_steps[i]);
    }

    if (steps.back().type != END) {
      steps.emplace_back(END);
    }
  }
}

bool Vehicle::has_start() const {
  return static_cast<bool>(start);
}

bool Vehicle::has_end() const {
  return static_cast<bool>(end);
}

bool Vehicle::has_same_locations(const Vehicle& other) const {
  bool same = (this->has_start() == other.has_start()) &&
              (this->has_end() == other.has_end());

  if (same && this->has_start()) {
    same = this->start.value() == other.start.value();
  }

  if (same && this->has_end()) {
    same = this->end.value() == other.end.value();
  }

  return same;
}

bool Vehicle::has_same_profile(const Vehicle& other) const {
  return (this->profile == other.profile) &&
         this->cost_wrapper.has_same_variable_costs(other.cost_wrapper);
}

bool Vehicle::cost_based_on_metrics() const {
  return cost_wrapper.cost_based_on_metrics();
}

Duration Vehicle::available_duration() const {
  const Duration available = tw.end - tw.start;

  const Duration breaks_duration =
    std::accumulate(breaks.begin(),
                    breaks.end(),
                    0,
                    [](auto sum, const auto& b) { return sum + b.service; });

  assert(breaks_duration <= available);

  return available - breaks_duration;
}

bool Vehicle::has_range_bounds() const {
  return max_travel_time != DEFAULT_MAX_TRAVEL_TIME ||
         max_distance != DEFAULT_MAX_DISTANCE;
}

Index Vehicle::break_rank(Id break_id) const {
  auto search = break_id_to_rank.find(break_id);
  assert(search != break_id_to_rank.end());
  return search->second;
}

} // namespace vroom
