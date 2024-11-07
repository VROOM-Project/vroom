#ifndef LOG_LOCAL_SEARCH_H
#define LOG_LOCAL_SEARCH_H

/*

This file is part of VROOM.

Copyright (c) 2015-2024, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/vroom/solution/solution.h"
#include "structures/vroom/solution_indicators.h"

namespace vroom::ls::log {

enum class EVENT {
  START,
  OPERATOR,
  LOCAL_MINIMA,
  JOB_ADDITION,
  RUIN,
  RECREATE,
  ROLLBACK
};

struct Step {
  TimePoint time_point;
  EVENT event;
  OperatorName operator_name;
  vroom::utils::SolutionIndicators indicators;
  std::optional<Solution> solution;
};

struct Dump {
  HeuristicParameters heuristic_parameters;
  std::vector<Step> steps;
};
} // namespace vroom::ls::log

#endif
