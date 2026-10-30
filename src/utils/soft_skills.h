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

} // namespace vroom

#endif
