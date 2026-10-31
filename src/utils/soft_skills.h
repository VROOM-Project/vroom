#ifndef SOFT_SKILLS_H
#define SOFT_SKILLS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/job.h"
#include "structures/vroom/soft_skills_options.h"
#include "structures/vroom/vehicle.h"

namespace vroom {

// Returns true iff the task's soft skills match the vehicle's
// according to mode. Empty task soft skills are neutral: no
// preference applies, so this is NOT a positive match under either
// mode.
inline bool soft_skill_match(const SoftSkills& task_skills,
                             const SoftSkills& vehicle_skills,
                             SoftSkillMatchMode mode) {
  if (task_skills.empty()) {
    return false;
  }

  if (mode == SoftSkillMatchMode::ALL) {
    for (const auto skill : task_skills) {
      if (!vehicle_skills.contains(skill)) {
        return false;
      }
    }
    return true;
  }

  // ANY mode.
  for (const auto skill : task_skills) {
    if (vehicle_skills.contains(skill)) {
      return true;
    }
  }
  return false;
}

// Flagging each vehicle in turn as matching the job's soft skills
// under the given mode.
inline std::vector<bool>
compute_matching_vehicles(const Job& job,
                          const std::vector<Vehicle>& vehicles,
                          SoftSkillMatchMode mode) {
  std::vector<bool> matching_vehicles;
  matching_vehicles.reserve(vehicles.size());
  for (const auto& vehicle : vehicles) {
    matching_vehicles.push_back(
      soft_skill_match(job.soft_skills, vehicle.soft_skills, mode));
  }
  return matching_vehicles;
}

// Pure threshold comparison predicate. Piecewise to avoid division:
// for a strictly positive reference, the preferred value passes when
// it stays below max_factor * reference (inclusive); a non-positive
// reference is only passed by a non-positive preferred value. Under
// ThresholdMode::COST the distance check always passes and vice
// versa. Missing distance data follows missing_policy: with
// SKIP_DISTANCE the distance check passes automatically, with REJECT
// the call returns false (parse-time validation should already
// prevent that case, this is defense in depth).
inline bool threshold_passes(Cost preferred_cost,
                             Cost reference_cost,
                             double max_cost_factor,
                             Distance preferred_distance,
                             Distance reference_distance,
                             double max_distance_factor,
                             ThresholdMode mode,
                             MissingDistancePolicy missing_policy,
                             bool distance_data_available) {
  // Piecewise check without division; only reached for metrics that
  // take part in the current mode.
  auto within_threshold = [](auto preferred, auto reference,
                             double max_factor) {
    if (reference > 0) {
      return static_cast<double>(preferred)
             <= max_factor * static_cast<double>(reference);
    }
    return preferred <= 0;
  };

  bool cost_ok = true;
  if (mode != ThresholdMode::DISTANCE) {
    cost_ok = within_threshold(preferred_cost, reference_cost,
                               max_cost_factor);
  }

  bool distance_ok = true;
  if (mode != ThresholdMode::COST) {
    if (!distance_data_available) {
      if (missing_policy == MissingDistancePolicy::REJECT) {
        return false;
      }
      // SKIP_DISTANCE: the distance check passes automatically.
    } else {
      distance_ok = within_threshold(preferred_distance,
                                     reference_distance,
                                     max_distance_factor);
    }
  }

  return cost_ok && distance_ok;
}

} // namespace vroom

#endif
