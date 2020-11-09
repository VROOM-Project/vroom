/*

This file is part of VROOM.

Copyright (c) 2015-2020, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/vehicle.h"
#include "utils/exception.h"

namespace vroom {

Vehicle::Vehicle(Id id,
                 const std::optional<Location>& start,
                 const std::optional<Location>& end,
                 const Amount& capacity,
                 const Skills& skills,
                 const TimeWindow& tw,
                 const std::vector<Break>& breaks,
                 const std::string& description,
                 const std::vector<InputStep>& steps)
  : id(id),
    start(start),
    end(end),
    capacity(capacity),
    skills(skills),
    tw(tw),
    breaks(breaks),
    description(description) {
  if (!static_cast<bool>(start) and !static_cast<bool>(end)) {
    throw Exception(ERROR::INPUT,
                    "No start or end specified for vehicle " +
                      std::to_string(id) + '.');
  }

  for (unsigned i = 0; i < breaks.size(); ++i) {
    const auto& b = breaks[i];
    if (break_id_to_rank.find(b.id) != break_id_to_rank.end()) {
      throw Exception(ERROR::INPUT,
                      "Duplicate break id: " + std::to_string(b.id) + ".");
    }
    break_id_to_rank[b.id] = i;
  }

  if (!steps.empty()) {
    // Populating input_steps. We rely on having start and end steps
    // in input, so just add them if they're missing (i.e. implicit
    // from vehicle definition).
    unsigned rank_after_start = 0;
    if (has_start()) {
      if (steps.front().type == STEP_TYPE::START) {
        input_steps.push_back(steps.front());
        rank_after_start = 1;
      } else {
        input_steps.emplace_back(STEP_TYPE::START);
      }
    }

    for (unsigned i = rank_after_start; i < steps.size(); ++i) {
      if (steps[i].type == STEP_TYPE::START) {
        throw Exception(ERROR::INPUT,
                        "Unexpected start in input steps for vehicle " +
                          std::to_string(id) + ".");
      }
      if (steps[i].type == STEP_TYPE::END and
          ((i != steps.size() - 1) or !has_end())) {
        throw Exception(ERROR::INPUT,
                        "Unexpected end in input steps for vehicle " +
                          std::to_string(id) + ".");
      }

      input_steps.push_back(steps[i]);
    }

    if (has_end() and input_steps.back().type != STEP_TYPE::END) {
      input_steps.emplace_back(STEP_TYPE::END);
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
  bool same = (this->has_start() == other.has_start()) and
              (this->has_end() == other.has_end());

  if (same and this->has_start()) {
    same = this->start.value() == other.start.value();
  }

  if (same and this->has_end()) {
    same = this->end.value() == other.end.value();
  }

  return same;
}

} // namespace vroom
