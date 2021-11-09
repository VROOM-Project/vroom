#ifndef VROOM_TYPEDEFS_H
#define VROOM_TYPEDEFS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
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
using Cost = uint32_t;
using Gain = int64_t;
using Distance = uint32_t;
using Duration = uint32_t;
using Coordinate = double;
using Capacity = int64_t;
using Skill = uint32_t;
using Priority = uint32_t;

// Type helpers.
using Coordinates = std::array<Coordinate, 2>;
using OptionalCoordinates = std::optional<Coordinates>;
using Skills = std::unordered_set<Skill>;
using TimePoint = std::chrono::high_resolution_clock::time_point;
using Timeout = std::optional<unsigned>;
using Deadline = std::optional<TimePoint>;

// Setting max value would cause trouble with further additions.
constexpr Cost INFINITE_COST = 3 * (std::numeric_limits<Cost>::max() / 4);

const std::string DEFAULT_PROFILE = "car";

constexpr Priority MAX_PRIORITY = 100;
constexpr double MAX_SPEED_FACTOR = 5.0;

// Available routing engines.
enum class ROUTER { OSRM, LIBOSRM, ORS, VALHALLA };

// Used to describe a routing server.
struct Server {
  std::string host;
  std::string port;

  Server() : host("0.0.0.0"), port("5000") {
  }

  Server(const std::string& host, const std::string& port)
    : host(host), port(port) {
  }
};

// Specific error statuses used when handling exceptions.
enum class ERROR { INTERNAL, INPUT, ROUTING };

// 'Single' job is a regular one-stop job without precedence
// constraints.
enum class JOB_TYPE { SINGLE, PICKUP, DELIVERY };

// Available location status.
enum class STEP_TYPE { START, JOB, BREAK, END };

// Heuristic options.
enum class HEURISTIC { BASIC, DYNAMIC, INIT_ROUTES };
enum class INIT { NONE, HIGHER_AMOUNT, NEAREST, FURTHEST, EARLIEST_DEADLINE };

struct HeuristicParameters {
  HEURISTIC heuristic;
  INIT init;
  float regret_coeff;

  constexpr HeuristicParameters(HEURISTIC heuristic,
                                INIT init,
                                float regret_coeff)
    : heuristic(heuristic), init(init), regret_coeff(regret_coeff) {
  }

  // Only makes sense for user-defined initial routes.
  constexpr HeuristicParameters(HEURISTIC heuristic)
    : heuristic(heuristic), init(INIT::NONE), regret_coeff(0) {
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
  MISSING_BREAK
};

} // namespace vroom

#endif
