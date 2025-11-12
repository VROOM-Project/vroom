#ifndef VROOM_TYPEDEFS_H
#define VROOM_TYPEDEFS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <limits>
#include <list>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace vroom {

// To easily differentiate variable types.
using Id = uint64_t;
using Index = uint16_t;
using UserCost = uint32_t;
using Cost = int64_t;
using UserDuration = uint32_t;
using Duration = int64_t;
using UserDistance = uint32_t;
using Distance = int64_t;
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
const std::string NO_TYPE = "";
const std::string DEFAULT_OSRM_SNAPPING_RADIUS = "35000";
constexpr double DEFAULT_LIBOSRM_SNAPPING_RADIUS = 35000;

// Our internal time measure is the hundredth of a second.
constexpr Duration DURATION_FACTOR = 100;

// Used to scale distances internally in a consistent way when used
// inside cost evaluations.
constexpr Distance DISTANCE_FACTOR = 360;

// Costs can be derived from travel times with a cost per hour for
// vehicles. So we scale all costs in order to not use floating point
// values while avoiding rounding issues internally.
constexpr Cost COST_FACTOR = 3600;
// This means a cost of one per second so that we default to
// outputting exact same values for duration and cost if per_hour
// values are not set.
constexpr UserCost DEFAULT_COST_PER_HOUR = 3600;
constexpr UserCost DEFAULT_COST_PER_TASK_HOUR = 0;
constexpr UserCost DEFAULT_COST_PER_KM = 0;

constexpr Priority MAX_PRIORITY = 100;
constexpr double MAX_SPEED_FACTOR = 5.0;
constexpr unsigned MAX_EXPLORATION_LEVEL = 5;

constexpr unsigned DEFAULT_EXPLORATION_LEVEL = 5;
constexpr unsigned DEFAULT_THREADS_NUMBER = 4;
constexpr unsigned MAX_ROUTING_THREADS = 32;

constexpr auto DEFAULT_MAX_TASKS = std::numeric_limits<size_t>::max();
constexpr auto DEFAULT_MAX_TRAVEL_TIME = std::numeric_limits<Duration>::max();
constexpr auto DEFAULT_MAX_DISTANCE = std::numeric_limits<Distance>::max();

// Available routing engines.
enum class ROUTER : std::uint8_t { OSRM, LIBOSRM, ORS, VALHALLA };

// Used to describe a routing server.
struct Server {
  std::string host;
  std::string port;
  std::string path;

  Server() : host("0.0.0.0"), port("5000") {
  }

  Server(std::string host, std::string port)
    : host(std::move(host)), port(std::move(port)) {
  }

  Server(std::string host, std::string port, std::string path)
    : host(std::move(host)), port(std::move(port)), path(std::move(path)) {
  }
};

// 'Single' job is a regular one-stop job without precedence
// constraints.
enum class JOB_TYPE : std::uint8_t { SINGLE, PICKUP, DELIVERY };

// Available location status.
enum class STEP_TYPE : std::uint8_t { START, JOB, BREAK, END };

// Heuristic options.
enum class HEURISTIC : std::uint8_t { BASIC, DYNAMIC };
enum class INIT : std::uint8_t {
  NONE,
  HIGHER_AMOUNT,
  NEAREST,
  FURTHEST,
  EARLIEST_DEADLINE
};
enum class SORT : std::uint8_t { AVAILABILITY, COST };

struct HeuristicParameters {
  HEURISTIC heuristic;
  INIT init;
  float regret_coeff;
  SORT sort;

  constexpr HeuristicParameters(HEURISTIC heuristic,
                                INIT init,
                                float regret_coeff,
                                SORT sort = SORT::AVAILABILITY)
    : heuristic(heuristic), init(init), regret_coeff(regret_coeff), sort(sort) {
  }
};

// Possible violations.
enum class VIOLATION : std::uint8_t {
  LEAD_TIME,
  DELAY,
  LOAD,
  MAX_TASKS,
  SKILLS,
  PRECEDENCE,
  MISSING_BREAK,
  MAX_TRAVEL_TIME,
  MAX_LOAD,
  MAX_DISTANCE
};

enum OperatorName : std::uint8_t {
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
  PriorityReplace,
  TSPFix,
  MAX
};

// Defined based on
// https://sonarcloud.io/organizations/vroom-project/rules?open=cpp%3AS6045&rule_key=cpp%3AS6045
struct StringHash {
  using is_transparent = void; // enables heterogenous lookup

  std::size_t operator()(std::string_view sv) const {
    const std::hash<std::string_view> hasher;
    return hasher(sv);
  }
};

using TypeToDurationMap =
  std::unordered_map<std::string, Duration, StringHash, std::equal_to<>>;

using TypeToUserDurationMap =
  std::unordered_map<std::string, UserDuration, StringHash, std::equal_to<>>;

namespace utils {
constexpr Duration scale_from_user_duration(UserDuration d) {
  return DURATION_FACTOR * static_cast<Duration>(d);
}

inline TypeToDurationMap
scale_from_user_duration(const TypeToUserDurationMap& user_duration_per_type) {
  TypeToDurationMap duration_per_type;

  std::ranges::transform(user_duration_per_type,
                         std::inserter(duration_per_type,
                                       duration_per_type.end()),
                         [](const auto& pair) {
                           return std::make_pair(pair.first,
                                                 scale_from_user_duration(
                                                   pair.second));
                         });

  return duration_per_type;
}

constexpr UserDuration scale_to_user_duration(Duration d) {
  assert(d <=
         scale_from_user_duration(std::numeric_limits<UserDuration>::max()));
  return static_cast<UserDuration>(d / DURATION_FACTOR);
}

constexpr Cost scale_from_user_cost(UserCost c) {
  return DURATION_FACTOR * COST_FACTOR * static_cast<Cost>(c);
}

constexpr UserCost scale_to_user_cost(Cost c) {
  assert(c <= scale_from_user_cost(std::numeric_limits<UserCost>::max()));
  return static_cast<UserCost>(c / (DURATION_FACTOR * COST_FACTOR));
}
} // namespace utils
} // namespace vroom

#endif
