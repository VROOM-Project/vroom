#ifndef VROOM_TYPEDEFS_H
#define VROOM_TYPEDEFS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <array>
#include <cassert>
#include <chrono>
#include <limits>
#include <list>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#ifdef _MSC_VER
// include support for "and"/"or"
#include <iso646.h>
#endif

namespace vroom {

// To easily differentiate variable types.
using Id = uint64_t;
using Index = uint16_t;
using UserCost = uint32_t;
using Cost = int64_t;
using UserDuration = uint32_t;
using Duration = int64_t;
using UserDistance = uint32_t;
using Coordinate = double;
using Capacity = int64_t;
using Skill = uint32_t;
using Priority = uint32_t;

// Type helpers.
struct Coordinates {
  Coordinate lon;
  Coordinate lat;
};

using OptionalCoordinates = std::optional<Coordinates>;
using Skills = std::unordered_set<Skill>;
using TimePoint = std::chrono::high_resolution_clock::time_point;
using Timeout = std::optional<std::chrono::milliseconds>;
using Deadline = std::optional<TimePoint>;

// Setting max value would cause trouble with further additions.
constexpr UserCost INFINITE_USER_COST =
  3 * (std::numeric_limits<UserCost>::max() / 4);

const std::string DEFAULT_PROFILE = "car";
const std::string DEFAULT_OSRM_SNAPPING_RADIUS = "35000";
constexpr double DEFAULT_LIBOSRM_SNAPPING_RADIUS = 35000;

// Our internal time measure is the hundredth of a second.
constexpr Duration DURATION_FACTOR = 100;

// Costs can be derived from travel times with a cost per hour for
// vehicles. So we scale all costs in order to not use floating point
// values while avoiding rounding issues internally.
constexpr Cost COST_FACTOR = 3600;
// This means a cost of one per second so that we default to
// outputting exact same values for duration and cost if per_hour
// values are not set.
constexpr UserCost DEFAULT_COST_PER_HOUR = 3600;

constexpr Priority MAX_PRIORITY = 100;
constexpr double MAX_SPEED_FACTOR = 5.0;
constexpr unsigned MAX_EXPLORATION_LEVEL = 5;

constexpr unsigned DEFAULT_EXPLORATION_LEVEL = 5;
constexpr unsigned DEFAULT_THREADS_NUMBER = 4;
constexpr Duration DEFAULT_MAX_TRAVEL_TIME =
  std::numeric_limits<Duration>::max();

// Available routing engines.
enum class ROUTER { OSRM, LIBOSRM, ORS, VALHALLA };

// Used to describe a routing server.
struct Server {
  std::string host;
  std::string port;
  std::string path;

  Server() : host("0.0.0.0"), port("5000"), path("") {
  }

  Server(std::string host, std::string port)
    : host(std::move(host)), port(std::move(port)), path("") {
  }
};

// 'Single' job is a regular one-stop job without precedence
// constraints.
enum class JOB_TYPE { SINGLE, PICKUP, DELIVERY };

// Available location status.
enum class STEP_TYPE { START, JOB, BREAK, END };

// Heuristic options.
enum class HEURISTIC { BASIC, DYNAMIC, INIT_ROUTES };
enum class INIT { NONE, HIGHEST_AMOUNT, NEAREST, FURTHEST, EARLIEST_DEADLINE };
enum class SORT { CAPACITY, COST };

struct HeuristicParameters {
  HEURISTIC heuristic;
  INIT init;
  float regret_coeff;
  SORT sort;

  constexpr HeuristicParameters(HEURISTIC heuristic,
                                INIT init,
                                float regret_coeff,
                                SORT sort = SORT::CAPACITY)
    : heuristic(heuristic), init(init), regret_coeff(regret_coeff), sort(sort) {
  }

  // Only makes sense for user-defined initial routes.
  constexpr HeuristicParameters(HEURISTIC heuristic)
    : heuristic(heuristic),
      init(INIT::NONE),
      regret_coeff(0),
      sort(SORT::CAPACITY) {
    assert(heuristic == HEURISTIC::INIT_ROUTES);
  }
};

// Possible violations.
enum class VIOLATION {
  LEAD_TIME,
  DELAY,
  LOAD,
  MAX_TASKS,
  SKILLS,
  PRECEDENCE,
  MISSING_BREAK,
  MAX_TRAVEL_TIME,
  MAX_LOAD
};

enum OperatorName {
  UnassignedExchange,
  CrossExchange,
  MixedExchange,
  TwoOpt,
  ReverseTwoOpt,
  Relocate,
  OrOpt,
  IntraExchange,
  IntraCrossExchange,
  IntraMixedExchange,
  IntraRelocate,
  IntraOrOpt,
  IntraTwoOpt,
  PDShift,
  RouteExchange,
  SwapStar,
  RouteSplit,
  MAX
};

namespace utils {
inline Duration scale_from_user_duration(UserDuration d) {
  return DURATION_FACTOR * static_cast<Duration>(d);
}

inline UserDuration scale_to_user_duration(Duration d) {
  return static_cast<UserDuration>(d / DURATION_FACTOR);
}

inline UserCost scale_to_user_cost(Cost d) {
  return static_cast<UserCost>(d / (DURATION_FACTOR * COST_FACTOR));
}
} // namespace utils

#ifdef LOG_LS_OPERATORS
namespace ls {
struct OperatorStats {
  unsigned tried_moves;
  unsigned applied_moves;

  OperatorStats() : tried_moves(0), applied_moves(0) {
  }

  OperatorStats(const unsigned tried_moves, const unsigned applied_moves)
    : tried_moves(tried_moves), applied_moves(applied_moves) {
  }
};
} // namespace ls
#endif

} // namespace vroom

#endif
