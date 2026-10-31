#ifndef SOFT_SKILLS_OPTIONS_H
#define SOFT_SKILLS_OPTIONS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <optional>

#include "structures/typedefs.h"
#include "structures/vroom/job.h"

namespace vroom {

// Available threshold basis for soft-skills matching.
enum class ThresholdMode : std::uint8_t { COST, DISTANCE, BOTH };

// Available policies when required distances are missing.
enum class MissingDistancePolicy : std::uint8_t { REJECT, SKIP_DISTANCE };

struct SoftSkillsOptions {
  SoftSkillMatchMode default_match_mode = SoftSkillMatchMode::ALL;
  ThresholdMode threshold_mode = ThresholdMode::COST;
  double max_cost_factor = 1.0;
  double max_distance_factor = 1.0;
  MissingDistancePolicy missing_distance_policy =
    MissingDistancePolicy::REJECT;
};

inline constexpr SoftSkillsOptions DEFAULT_SOFT_SKILLS_OPTIONS{};

// A job-level match mode overrides the default from the options.
inline SoftSkillMatchMode effective_match_mode(const Job& job,
                                               const SoftSkillsOptions& opts) {
  return job.soft_skill_match_mode.value_or(opts.default_match_mode);
}

} // namespace vroom

#endif
