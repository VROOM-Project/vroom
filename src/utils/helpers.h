#ifndef HELPERS_H
#define HELPERS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>
#include <sstream>

#include "structures/typedefs.h"
#include "structures/vroom/raw_route.h"
#include "structures/vroom/tw_route.h"
#include "utils/exception.h"

namespace vroom {
namespace utils {

using RawSolution = std::vector<RawRoute>;
using TWSolution = std::vector<TWRoute>;

inline TimePoint now() {
  return std::chrono::high_resolution_clock::now();
}

inline Cost add_without_overflow(Cost a, Cost b) {
  if (a > std::numeric_limits<Cost>::max() - b) {
    throw Exception(ERROR::INPUT,
                    "Too high cost values, stopping to avoid overflowing.");
  }
  return a + b;
}

inline INIT get_init(const std::string& s) {
  if (s == "NONE") {
    return INIT::NONE;
  } else if (s == "HIGHER_AMOUNT") {
    return INIT::HIGHER_AMOUNT;
  } else if (s == "NEAREST") {
    return INIT::NEAREST;
  } else if (s == "FURTHEST") {
    return INIT::FURTHEST;
  } else if (s == "EARLIEST_DEADLINE") {
    return INIT::EARLIEST_DEADLINE;
  } else {
    throw Exception(ERROR::INPUT,
                    "Invalid heuristic parameter in command-line.");
  }
}

inline HeuristicParameters str_to_heuristic_param(const std::string& s) {
  // Split command-line string describing parameters.
  constexpr char delimiter = ',';
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }

  if (tokens.size() != 3 or tokens[0].size() != 1) {
    throw Exception(ERROR::INPUT,
                    "Invalid heuristic parameter in command-line.");
  }

  auto init = get_init(tokens[1]);
  try {
    auto h = std::stoul(tokens[0]);

    if (h != 0 and h != 1) {
      throw Exception(ERROR::INPUT,
                      "Invalid heuristic parameter in command-line.");
    }

    auto regret_coeff = std::stof(tokens[2]);
    if (regret_coeff < 0) {
      throw Exception(ERROR::INPUT,
                      "Invalid heuristic parameter in command-line.");
    }

    return HeuristicParameters(static_cast<HEURISTIC>(h), init, regret_coeff);
  } catch (const std::exception& e) {
    throw Exception(ERROR::INPUT,
                    "Invalid heuristic parameter in command-line.");
  }
}

// Compute cost of adding job with rank job_rank in given route at
// given rank for vehicle v.
inline Gain addition_cost(const Input& input,
                          Index job_rank,
                          const Vehicle& v,
                          const std::vector<Index>& route,
                          Index rank) {
  assert(rank <= route.size());

  Index job_index = input.jobs[job_rank].index();
  Gain previous_cost = 0;
  Gain next_cost = 0;
  Gain old_edge_cost = 0;

  if (rank == route.size()) {
    if (route.size() == 0) {
      // Adding job to an empty route.
      if (v.has_start()) {
        previous_cost = v.cost(v.start.value().index(), job_index);
      }
      if (v.has_end()) {
        next_cost = v.cost(job_index, v.end.value().index());
      }
    } else {
      // Adding job past the end after a real job.
      auto p_index = input.jobs[route[rank - 1]].index();
      previous_cost = v.cost(p_index, job_index);
      if (v.has_end()) {
        auto n_index = v.end.value().index();
        old_edge_cost = v.cost(p_index, n_index);
        next_cost = v.cost(job_index, n_index);
      }
    }
  } else {
    // Adding before one of the jobs.
    auto n_index = input.jobs[route[rank]].index();
    next_cost = v.cost(job_index, n_index);

    if (rank == 0) {
      if (v.has_start()) {
        auto p_index = v.start.value().index();
        previous_cost = v.cost(p_index, job_index);
        old_edge_cost = v.cost(p_index, n_index);
      }
    } else {
      auto p_index = input.jobs[route[rank - 1]].index();
      previous_cost = v.cost(p_index, job_index);
      old_edge_cost = v.cost(p_index, n_index);
    }
  }

  return previous_cost + next_cost - old_edge_cost;
}

// Compute cost of adding pickup with rank job_rank and associated
// delivery (with rank job_rank + 1) in given route for vehicle
// v. Pickup is inserted at pickup_rank in route and delivery is
// inserted at delivery_rank in route **with pickup**.
inline Gain addition_cost(const Input& input,
                          Index job_rank,
                          const Vehicle& v,
                          const std::vector<Index>& route,
                          Index pickup_rank,
                          Index delivery_rank) {
  assert(pickup_rank < delivery_rank and delivery_rank <= route.size() + 1);

  // Start with pickup cost.
  auto cost = addition_cost(input, job_rank, v, route, pickup_rank);

  if (delivery_rank == pickup_rank + 1) {
    // Delivery is inserted just after pickup.
    Index p_index = input.jobs[job_rank].index();
    Index d_index = input.jobs[job_rank + 1].index();
    cost += v.cost(p_index, d_index);

    Gain after_delivery = 0;
    Gain remove_after_pickup = 0;

    if (pickup_rank == route.size()) {
      // Addition at the end of a route.
      if (v.has_end()) {
        after_delivery = v.cost(d_index, v.end.value().index());
        remove_after_pickup = v.cost(p_index, v.end.value().index());
      }
    } else {
      // There is a job after insertion.
      Index next_index = input.jobs[route[pickup_rank]].index();
      after_delivery = v.cost(d_index, next_index);
      remove_after_pickup = v.cost(p_index, next_index);
    }

    cost += after_delivery;
    cost -= remove_after_pickup;
  } else {
    // Delivery is further away so edges sets for pickup and delivery
    // addition are disjoint.
    cost += addition_cost(input, job_rank + 1, v, route, delivery_rank - 1);
  }

  return cost;
}

// Helper function for SwapStar operator, computing part of the cost
// for in-place replacing of job at rank in route with job at
// job_rank.
inline Gain in_place_delta_cost(const Input& input,
                                Index job_rank,
                                const Vehicle& v,
                                const std::vector<Index>& route,
                                Index rank) {
  assert(!route.empty());
  Index new_index = input.jobs[job_rank].index();

  Gain new_previous_cost = 0;
  Gain new_next_cost = 0;
  std::optional<Index> p_index;
  std::optional<Index> n_index;

  if (rank == 0) {
    if (v.has_start()) {
      p_index = v.start.value().index();
      new_previous_cost = v.cost(p_index.value(), new_index);
    }
  } else {
    p_index = input.jobs[route[rank - 1]].index();
    new_previous_cost = v.cost(p_index.value(), new_index);
  }

  if (rank == route.size() - 1) {
    if (v.has_end()) {
      n_index = v.end.value().index();
      new_next_cost = v.cost(new_index, n_index.value());
    }
  } else {
    n_index = input.jobs[route[rank + 1]].index();
    new_next_cost = v.cost(new_index, n_index.value());
  }

  Gain old_virtual_cost = 0;
  if (p_index and n_index) {
    old_virtual_cost = v.cost(p_index.value(), n_index.value());
  }

  return new_previous_cost + new_next_cost - old_virtual_cost;
}

inline Cost priority_sum_for_route(const Input& input,
                                   const std::vector<Index>& route) {
  return std::accumulate(route.begin(),
                         route.end(),
                         0,
                         [&](auto sum, auto job_rank) {
                           return sum + input.jobs[job_rank].priority;
                         });
}

inline Cost route_cost_for_vehicle(const Input& input,
                                   Index vehicle_rank,
                                   const std::vector<Index>& route) {
  const auto& v = input.vehicles[vehicle_rank];
  auto cost = 0;

  if (route.size() > 0) {
    if (v.has_start()) {
      cost +=
        v.cost(v.start.value().index(), input.jobs[route.front()].index());
    }

    Index previous = route.front();
    for (auto it = ++route.cbegin(); it != route.cend(); ++it) {
      cost += v.cost(input.jobs[previous].index(), input.jobs[*it].index());
      previous = *it;
    }

    if (v.has_end()) {
      cost += v.cost(input.jobs[route.back()].index(), v.end.value().index());
    }
  }

  return cost;
}

inline void check_precedence(const Input& input,
                             std::unordered_set<Index>& expected_delivery_ranks,
                             Index job_rank) {
  switch (input.jobs[job_rank].type) {
  case JOB_TYPE::SINGLE:
    break;
  case JOB_TYPE::PICKUP:
    expected_delivery_ranks.insert(job_rank + 1);
    break;
  case JOB_TYPE::DELIVERY:
    // Associated pickup has been done before.
    auto search = expected_delivery_ranks.find(job_rank);
    assert(search != expected_delivery_ranks.end());
    expected_delivery_ranks.erase(search);
    break;
  }
}

inline void check_tws(const std::vector<TimeWindow>& tws) {
  if (tws.size() == 0) {
    throw Exception(ERROR::INPUT, "Empty time-windows.");
  }

  if (tws.size() > 1) {
    for (std::size_t i = 0; i < tws.size() - 1; ++i) {
      if (tws[i + 1].start <= tws[i].end) {
        throw Exception(ERROR::INPUT, "Unsorted or overlapping time-windows.");
      }
    }
  }
}

inline Solution format_solution(const Input& input,
                                const RawSolution& raw_routes) {
  std::vector<Route> routes;

  // All job ranks start with unassigned status.
  std::unordered_set<Index> unassigned_ranks;
  for (unsigned i = 0; i < input.jobs.size(); ++i) {
    unassigned_ranks.insert(i);
  }

  for (std::size_t i = 0; i < raw_routes.size(); ++i) {
    const auto& route = raw_routes[i].route;
    if (route.empty()) {
      continue;
    }
    const auto& v = input.vehicles[i];

    assert(route.size() <= v.max_tasks);

    auto previous_location = (v.has_start())
                               ? v.start.value().index()
                               : std::numeric_limits<Index>::max();
    Cost cost = 0;
    Duration duration = 0;
    Duration setup = 0;
    Duration service = 0;
    Priority priority = 0;
    Amount sum_pickups(input.zero_amount());
    Amount sum_deliveries(input.zero_amount());
#ifndef NDEBUG
    std::unordered_set<Index> expected_delivery_ranks;
#endif
    Amount current_load = raw_routes[i].get_startup_load();
    assert(current_load <= v.capacity);

    // Steps for current route.
    std::vector<Step> steps;

    Duration ETA = 0;
    const auto& first_job = input.jobs[route.front()];

    // Handle start.
    const auto start_loc = v.has_start() ? v.start.value() : first_job.location;
    steps.emplace_back(STEP_TYPE::START, start_loc, current_load);
    if (v.has_start()) {
      const auto travel =
        v.duration(v.start.value().index(), first_job.index());
      ETA += travel;
      duration += travel;
      cost += v.cost(v.start.value().index(), first_job.index());
    }

    // Handle jobs.
    assert(input.vehicle_ok_with_job(i, route.front()));

    const auto first_job_setup =
      (first_job.index() == previous_location) ? 0 : first_job.setup;
    setup += first_job_setup;
    previous_location = first_job.index();

    service += first_job.service;
    priority += first_job.priority;

    current_load += first_job.pickup;
    current_load -= first_job.delivery;
    sum_pickups += first_job.pickup;
    sum_deliveries += first_job.delivery;
    assert(current_load <= v.capacity);

#ifndef NDEBUG
    check_precedence(input, expected_delivery_ranks, route.front());
#endif

    steps.emplace_back(first_job, first_job_setup, current_load);
    auto& first = steps.back();
    first.duration = ETA;
    first.arrival = ETA;
    ETA += (first.setup + first.service);
    unassigned_ranks.erase(route.front());

    for (std::size_t r = 0; r < route.size() - 1; ++r) {
      assert(input.vehicle_ok_with_job(i, route[r + 1]));
      const auto travel = v.duration(input.jobs[route[r]].index(),
                                     input.jobs[route[r + 1]].index());
      ETA += travel;
      duration += travel;

      cost +=
        v.cost(input.jobs[route[r]].index(), input.jobs[route[r + 1]].index());

      auto& current_job = input.jobs[route[r + 1]];

      const auto current_setup =
        (current_job.index() == previous_location) ? 0 : current_job.setup;
      setup += current_setup;
      previous_location = current_job.index();

      service += current_job.service;
      priority += current_job.priority;

      current_load += current_job.pickup;
      current_load -= current_job.delivery;
      sum_pickups += current_job.pickup;
      sum_deliveries += current_job.delivery;
      assert(current_load <= v.capacity);

#ifndef NDEBUG
      check_precedence(input, expected_delivery_ranks, route[r + 1]);
#endif

      steps.emplace_back(current_job, current_setup, current_load);
      auto& current = steps.back();
      current.duration = duration;
      current.arrival = ETA;
      ETA += (current_setup + current.service);
      unassigned_ranks.erase(route[r + 1]);
    }

    // Handle end.
    const auto& last_job = input.jobs[route.back()];
    const auto end_loc = v.has_end() ? v.end.value() : last_job.location;
    steps.emplace_back(STEP_TYPE::END, end_loc, current_load);
    if (v.has_end()) {
      const auto travel = v.duration(last_job.index(), v.end.value().index());
      ETA += travel;
      duration += travel;
      cost += v.cost(last_job.index(), v.end.value().index());
    }
    steps.back().duration = duration;
    steps.back().arrival = ETA;

    assert(expected_delivery_ranks.empty());

    routes.emplace_back(v.id,
                        std::move(steps),
                        cost,
                        setup,
                        service,
                        duration,
                        0,
                        priority,
                        sum_deliveries,
                        sum_pickups,
                        v.profile,
                        v.description);
  }

  // Handle unassigned jobs.
  std::vector<Job> unassigned_jobs;
  std::transform(unassigned_ranks.begin(),
                 unassigned_ranks.end(),
                 std::back_inserter(unassigned_jobs),
                 [&](auto j) { return input.jobs[j]; });

  return Solution(0,
                  input.zero_amount().size(),
                  std::move(routes),
                  std::move(unassigned_jobs));
}

inline Route format_route(const Input& input,
                          const TWRoute& tw_r,
                          std::unordered_set<Index>& unassigned_ranks) {
  const auto& v = input.vehicles[tw_r.vehicle_rank];

  assert(tw_r.size() <= v.max_tasks);

  // ETA logic: aim at earliest possible arrival then determine latest
  // possible start time in order to minimize waiting times.
  Duration step_start = tw_r.earliest_end;
  Duration backward_wt = 0;
  std::optional<Location> first_location;
  std::optional<Location> last_location;

  if (v.has_end()) {
    first_location = v.end.value();
    last_location = v.end.value();
  }

  for (std::size_t r = tw_r.route.size(); r > 0; --r) {
    const auto& previous_job = input.jobs[tw_r.route[r - 1]];

    if (!last_location.has_value()) {
      last_location = previous_job.location;
    }
    first_location = previous_job.location;

    // Remaining travel time is the time between two jobs, except for
    // last rank where it depends whether the vehicle has an end or
    // not.
    Duration remaining_travel_time =
      (r < tw_r.route.size())
        ? v.duration(previous_job.index(), input.jobs[tw_r.route[r]].index())
        : (v.has_end())
            ? v.duration(previous_job.index(), v.end.value().index())
            : 0;

    // Take into account timing constraints for breaks before current
    // job.
    assert(tw_r.breaks_at_rank[r] <= tw_r.breaks_counts[r]);
    Index break_rank = tw_r.breaks_counts[r];
    for (Index i = 0; i < tw_r.breaks_at_rank[r]; ++i) {
      --break_rank;
      const auto& b = v.breaks[break_rank];
      assert(b.service <= step_start);
      step_start -= b.service;

      const auto b_tw =
        std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
          return tw.start <= step_start;
        });
      assert(b_tw != b.tws.rend());

      if (b_tw->end < step_start) {
        auto margin = step_start - b_tw->end;
        if (margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          backward_wt += (margin - remaining_travel_time);
          remaining_travel_time = 0;
        }

        step_start = b_tw->end;
      }
    }

    bool same_location = (r > 1 and input.jobs[tw_r.route[r - 2]].index() ==
                                      previous_job.index()) or
                         (r == 1 and v.has_start() and
                          v.start.value().index() == previous_job.index());
    const auto current_setup = (same_location) ? 0 : previous_job.setup;

    Duration diff =
      current_setup + previous_job.service + remaining_travel_time;

    assert(diff <= step_start);
    Duration candidate_start = step_start - diff;
    assert(tw_r.earliest[r - 1] <= candidate_start);

    const auto j_tw =
      std::find_if(previous_job.tws.rbegin(),
                   previous_job.tws.rend(),
                   [&](const auto& tw) { return tw.start <= candidate_start; });
    assert(j_tw != previous_job.tws.rend());

    step_start = std::min(candidate_start, j_tw->end);
    if (step_start < candidate_start) {
      backward_wt += (candidate_start - step_start);
    }
    assert(previous_job.is_valid_start(step_start));
  }

#ifndef NDEBUG
  std::unordered_set<Index> expected_delivery_ranks;
#endif
  Amount current_load = tw_r.get_startup_load();
  assert(current_load <= v.capacity);

  // Steps for current route.
  std::vector<Step> steps;

  // Now pack everything ASAP based on first job start date.
  Duration remaining_travel_time =
    (v.has_start())
      ? v.duration(v.start.value().index(), input.jobs[tw_r.route[0]].index())
      : 0;

  // Take into account timing constraints for breaks before first job.
  assert(tw_r.breaks_at_rank[0] <= tw_r.breaks_counts[0]);
  Index break_rank = tw_r.breaks_counts[0];
  for (Index r = 0; r < tw_r.breaks_at_rank[0]; ++r) {
    --break_rank;
    const auto& b = v.breaks[break_rank];
    assert(b.service <= step_start);
    step_start -= b.service;

    const auto b_tw =
      std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
        return tw.start <= step_start;
      });
    assert(b_tw != b.tws.rend());

    if (b_tw->end < step_start) {
      auto margin = step_start - b_tw->end;
      if (margin < remaining_travel_time) {
        remaining_travel_time -= margin;
      } else {
        backward_wt += (margin - remaining_travel_time);
        remaining_travel_time = 0;
      }

      step_start = b_tw->end;
    }
  }

  if (v.has_start()) {
    first_location = v.start.value();
    assert(remaining_travel_time <= step_start);
    step_start -= remaining_travel_time;
  }

  assert(first_location.has_value() and last_location.has_value());
  steps.emplace_back(STEP_TYPE::START, first_location.value(), current_load);
  assert(v.tw.contains(step_start));
  steps.back().arrival = step_start;

  auto previous_location = (v.has_start()) ? v.start.value().index()
                                           : std::numeric_limits<Index>::max();

  // Values summed up while going through the route.
  Cost cost = 0;
  Duration duration = 0;
  Duration setup = 0;
  Duration service = 0;
  Duration forward_wt = 0;
  Priority priority = 0;
  Amount sum_pickups(input.zero_amount());
  Amount sum_deliveries(input.zero_amount());

  //  Go through the whole route again to set jobs/breaks ASAP given
  // the latest possible start time.
  Duration travel_time =
    v.has_start()
      ? v.duration(v.start.value().index(), input.jobs[tw_r.route[0]].index())
      : 0;

  Cost current_cost = v.has_start() ? v.cost(v.start.value().index(),
                                             input.jobs[tw_r.route[0]].index())
                                    : 0;

  for (std::size_t r = 0; r < tw_r.route.size(); ++r) {
    assert(input.vehicle_ok_with_job(tw_r.vehicle_rank, tw_r.route[r]));
    const auto& current_job = input.jobs[tw_r.route[r]];

    if (r > 0) {
      // For r == 0, travel_time already holds the relevant value
      // depending on whether there is a start.
      travel_time =
        v.duration(input.jobs[tw_r.route[r - 1]].index(), current_job.index());
      current_cost =
        v.cost(input.jobs[tw_r.route[r - 1]].index(), current_job.index());
    }

    // Handles breaks before this job.
    assert(tw_r.breaks_at_rank[r] <= tw_r.breaks_counts[r]);
    Index break_rank = tw_r.breaks_counts[r] - tw_r.breaks_at_rank[r];

    for (Index i = 0; i < tw_r.breaks_at_rank[r]; ++i, ++break_rank) {
      const auto& b = v.breaks[break_rank];

      steps.emplace_back(b, current_load);
      auto& current_break = steps.back();

      const auto b_tw =
        std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
          return step_start <= tw.end;
        });
      assert(b_tw != b.tws.end());

      if (step_start < b_tw->start) {
        auto margin = b_tw->start - step_start;
        if (margin <= travel_time) {
          // Part of the remaining travel time is spent before this
          // break, filling the whole margin.
          duration += margin;
          travel_time -= margin;
          current_break.arrival = b_tw->start;
        } else {
          // The whole remaining travel time is spent before this
          // break, not filling the whole margin.

          Duration wt = margin - travel_time;
          forward_wt += wt;

          current_break.arrival = step_start + travel_time;
          current_break.waiting_time = wt;

          duration += travel_time;
          travel_time = 0;
        }

        step_start = b_tw->start;
      } else {
        current_break.arrival = step_start;
      }

      assert(
        b.is_valid_start(current_break.arrival + current_break.waiting_time));

      current_break.duration = duration;

      auto& current_service = b.service;
      service += current_service;
      step_start += current_service;
    }

    // Back to current job.
    duration += travel_time;
    cost += current_cost;
    service += current_job.service;
    priority += current_job.priority;

    const auto current_setup =
      (current_job.index() == previous_location) ? 0 : current_job.setup;
    setup += current_setup;
    previous_location = current_job.index();

    current_load += current_job.pickup;
    current_load -= current_job.delivery;
    sum_pickups += current_job.pickup;
    sum_deliveries += current_job.delivery;
    assert(current_load <= v.capacity);

#ifndef NDEBUG
    check_precedence(input, expected_delivery_ranks, tw_r.route[r]);
#endif

    steps.emplace_back(current_job, current_setup, current_load);
    auto& current = steps.back();
    current.duration = duration;

    step_start += travel_time;
    assert(step_start <= tw_r.latest[r]);

    current.arrival = step_start;

    const auto j_tw =
      std::find_if(current_job.tws.begin(),
                   current_job.tws.end(),
                   [&](const auto& tw) { return step_start <= tw.end; });
    assert(j_tw != current_job.tws.end());

    if (step_start < j_tw->start) {
      Duration wt = j_tw->start - step_start;
      current.waiting_time = wt;
      forward_wt += wt;

      step_start = j_tw->start;
    }
    assert(current_job.is_valid_start(current.arrival + current.waiting_time));

    step_start += (current_setup + current_job.service);

    unassigned_ranks.erase(tw_r.route[r]);
  }

  // Handle breaks after last job.
  travel_time =
    (v.has_end())
      ? v.duration(input.jobs[tw_r.route.back()].index(), v.end.value().index())
      : 0;
  current_cost = (v.has_end()) ? v.cost(input.jobs[tw_r.route.back()].index(),
                                        v.end.value().index())
                               : 0;

  auto r = tw_r.route.size();
  assert(tw_r.breaks_at_rank[r] <= tw_r.breaks_counts[r]);
  break_rank = tw_r.breaks_counts[r] - tw_r.breaks_at_rank[r];

  for (Index i = 0; i < tw_r.breaks_at_rank[r]; ++i, ++break_rank) {
    const auto& b = v.breaks[break_rank];

    steps.emplace_back(b, current_load);
    auto& current_break = steps.back();

    const auto b_tw =
      std::find_if(b.tws.begin(), b.tws.end(), [&](const auto& tw) {
        return step_start <= tw.end;
      });
    assert(b_tw != b.tws.end());

    if (step_start < b_tw->start) {
      auto margin = b_tw->start - step_start;
      if (margin <= travel_time) {
        // Part of the remaining travel time is spent before this
        // break, filling the whole margin.
        duration += margin;
        travel_time -= margin;
        current_break.arrival = b_tw->start;
      } else {
        // The whole remaining travel time is spent before this
        // break, not filling the whole margin.

        Duration wt = margin - travel_time;
        forward_wt += wt;

        current_break.arrival = step_start + travel_time;
        current_break.waiting_time = wt;

        duration += travel_time;
        travel_time = 0;
      }

      step_start = b_tw->start;
    } else {
      current_break.arrival = step_start;
    }

    assert(
      b.is_valid_start(current_break.arrival + current_break.waiting_time));

    current_break.duration = duration;

    auto& current_service = b.service;
    service += current_service;
    step_start += current_service;
  }

  steps.emplace_back(STEP_TYPE::END, last_location.value(), current_load);
  if (v.has_end()) {
    duration += travel_time;
    cost += current_cost;
    step_start += travel_time;
  }
  steps.back().duration = duration;
  assert(v.tw.contains(step_start));
  steps.back().arrival = step_start;

  assert(step_start == tw_r.earliest_end);
  assert(forward_wt == backward_wt);

  assert(steps.back().arrival + steps.back().waiting_time +
           steps.back().service ==
         steps.front().arrival + duration + setup + service + forward_wt);

  assert(expected_delivery_ranks.empty());

  return Route(v.id,
               std::move(steps),
               cost,
               setup,
               service,
               duration,
               forward_wt,
               priority,
               sum_deliveries,
               sum_pickups,
               v.profile,
               v.description);
}

inline Solution format_solution(const Input& input,
                                const TWSolution& tw_routes) {
  std::vector<Route> routes;

  // All job ranks start with unassigned status.
  std::unordered_set<Index> unassigned_ranks;
  for (unsigned i = 0; i < input.jobs.size(); ++i) {
    unassigned_ranks.insert(i);
  }

  for (const auto& tw_route : tw_routes) {
    if (!tw_route.empty()) {
      routes.push_back(format_route(input, tw_route, unassigned_ranks));
    }
  }

  // Handle unassigned jobs.
  std::vector<Job> unassigned_jobs;
  std::transform(unassigned_ranks.begin(),
                 unassigned_ranks.end(),
                 std::back_inserter(unassigned_jobs),
                 [&](auto j) { return input.jobs[j]; });

  return Solution(0,
                  input.zero_amount().size(),
                  std::move(routes),
                  std::move(unassigned_jobs));
}

inline unsigned get_code(ERROR e) {
  unsigned code = 0;
  switch (e) {
  case ERROR::INTERNAL:
    code = 1;
    break;
  case ERROR::INPUT:
    code = 2;
    break;
  case ERROR::ROUTING:
    code = 3;
    break;
  }

  return code;
}

} // namespace utils
} // namespace vroom

#endif
