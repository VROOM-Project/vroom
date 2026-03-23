/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <optional>
#include <sstream>

#include "utils/helpers.h"

namespace vroom::utils {

TimePoint now() {
  return std::chrono::high_resolution_clock::now();
}

Amount max_amount(std::size_t size) {
  Amount max(size);
  for (std::size_t i = 0; i < size; ++i) {
    max[i] = std::numeric_limits<Capacity>::max();
  }
  return max;
}

INIT get_init(std::string_view s) {
  using enum INIT;
  if (s == "NONE") {
    return NONE;
  }
  if (s == "HIGHER_AMOUNT") {
    return HIGHER_AMOUNT;
  }
  if (s == "NEAREST") {
    return NEAREST;
  }
  if (s == "FURTHEST") {
    return FURTHEST;
  }
  if (s == "EARLIEST_DEADLINE") {
    return EARLIEST_DEADLINE;
  }
  throw InputException("Invalid heuristic parameter in command-line.");
}

SORT get_sort(std::string_view s) {
  if (s == "AVAILABILITY") {
    return SORT::AVAILABILITY;
  }
  if (s == "COST") {
    return SORT::COST;
  }
  throw InputException("Invalid heuristic parameter in command-line.");
}

HeuristicParameters str_to_heuristic_param(const std::string& s) {
  // Split command-line string describing parameters.
  constexpr char delimiter = ';';
  std::vector<std::string> tokens;
  tokens.reserve(4);
  std::string token;
  std::istringstream tokenStream(s);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(token);
  }

  if ((tokens.size() != 3 && tokens.size() != 4) || tokens[0].size() != 1) {
    throw InputException("Invalid heuristic parameter in command-line.");
  }

  auto init = get_init(tokens[1]);
  auto sort = (tokens.size() == 3) ? SORT::AVAILABILITY : get_sort(tokens[3]);

  try {
    auto h = std::stoul(tokens[0]);

    if (h != 0 && h != 1) {
      throw InputException("Invalid heuristic parameter in command-line.");
    }

    auto regret_coeff = std::stof(tokens[2]);
    if (regret_coeff < 0) {
      throw InputException("Invalid heuristic parameter in command-line.");
    }

    return HeuristicParameters(static_cast<HEURISTIC>(h),
                               init,
                               regret_coeff,
                               sort);
  } catch (const std::exception&) {
    throw InputException("Invalid heuristic parameter in command-line.");
  }
}

Priority priority_sum_for_route(const Input& input,
                                const std::vector<Index>& route) {
  return std::accumulate(route.begin(),
                         route.end(),
                         0,
                         [&](auto sum, auto job_rank) {
                           return sum + input.jobs[job_rank].priority;
                         });
}

Eval route_eval_for_vehicle(const Input& input,
                            Index v_rank,
                            const std::vector<Index>::const_iterator first_job,
                            const std::vector<Index>::const_iterator last_job) {
  const auto& v = input.vehicles[v_rank];
  Eval eval;

  if (first_job != last_job) {
    eval.cost += v.fixed_cost();

    if (v.has_start()) {
      eval += v.eval(v.start.value().index(), input.jobs[*first_job].index());
    }

    Index previous = *first_job;
    for (auto it = std::next(first_job); it != last_job; ++it) {
      eval += v.eval(input.jobs[previous].index(), input.jobs[*it].index());
      previous = *it;
    }

    if (v.has_end()) {
      eval += v.eval(input.jobs[previous].index(), v.end.value().index());
    }
  }

  return eval;
}

Eval route_eval_for_vehicle(const Input& input,
                            Index v_rank,
                            const std::vector<Index>& route) {
  return route_eval_for_vehicle(input, v_rank, route.begin(), route.end());
}

Cost pickup_approach_penalty(const Input& input,
                             Index v_rank,
                             const std::vector<Index>& route) {
  const auto& v = input.vehicles[v_rank];
  const double I = v.initial_pickup_cost_multiplier;
  const double N = v.non_initial_pickup_cost_multiplier;
  if (I == 1.0 && N == 1.0) {
    return 0;
  }

  Cost penalty = 0;
  bool first_pickup_seen = false;
  std::optional<Index> prev_index;
  if (v.has_start()) {
    prev_index = v.start.value().index();
  }

  for (const auto jr : route) {
    const auto& job = input.jobs[jr];
    if (job.type == JOB_TYPE::PICKUP && prev_index.has_value()) {
      const Cost edge_cost = v.cost(prev_index.value(), job.index());
      if (!first_pickup_seen) {
        penalty += static_cast<Cost>(std::round(edge_cost * (I - 1.0)));
        first_pickup_seen = true;
      } else {
        penalty += static_cast<Cost>(std::round(edge_cost * (N - 1.0)));
      }
    }
    prev_index = job.index();
  }
  return penalty;
}

#ifndef NDEBUG
void check_precedence(const Input& input,
                      std::unordered_set<Index>& expected_delivery_ranks,
                      Index job_rank) {
  switch (input.jobs[job_rank].type) {
    using enum JOB_TYPE;
  case SINGLE:
    break;
  case PICKUP:
    expected_delivery_ranks.insert(job_rank + 1);
    break;
  case DELIVERY:
    // Associated pickup has been done before.
    auto search = expected_delivery_ranks.find(job_rank);
    assert(search != expected_delivery_ranks.end());
    expected_delivery_ranks.erase(search);
    break;
  }
}
#endif

void check_tws(const std::vector<TimeWindow>& tws,
               const Id id,
               const std::string& type) {
  if (tws.empty()) {
    throw InputException(
      std::format("Empty time windows for {} {}.", type, id));
  }

  if (tws.size() > 1) {
    for (std::size_t i = 0; i < tws.size() - 1; ++i) {
      if (tws[i + 1].start <= tws[i].end) {
        throw InputException(
          std::format("Unsorted or overlapping time-windows for {} {}.",
                      type,
                      id));
      }
    }
  }
}

void check_priority(const Priority priority,
                    const Id id,
                    const std::string& type) {
  if (priority > MAX_PRIORITY) {
    throw InputException(
      std::format("Invalid priority value for {} {}.", type, id));
  }
}

void check_no_empty_keys(const TypeToDurationMap& type_to_duration,
                         const Id id,
                         const std::string& type,
                         const std::string& key_name) {
  if (std::ranges::any_of(type_to_duration, [](const auto& pair) {
        return pair.first.empty();
      })) {
    throw InputException(
      std::format("Empty type in {} for {} {}.", key_name, type, id));
  }
}

inline std::vector<Job> get_unassigned_jobs_from_ranks(
  const Input& input,
  const std::unordered_set<Index>& unassigned_ranks) {
  std::vector<Job> unassigned_jobs;
  std::ranges::transform(unassigned_ranks,
                         std::back_inserter(unassigned_jobs),
                         [&](auto j) { return input.jobs[j]; });

  return unassigned_jobs;
}

Solution format_solution(const Input& input, const RawSolution& raw_routes) {
  std::vector<Route> routes;
  routes.reserve(raw_routes.size());

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

    // Skip routes containing any job incompatible with the vehicle.
    const bool all_compatible = std::ranges::all_of(
      route, [&](Index j_rank) { return input.vehicle_ok_with_job(i, j_rank); });
    if (!all_compatible) {
      // Leave those jobs in unassigned_ranks.
      continue;
    }

    // Enforce first-leg distance limit for vehicles without pre-defined steps.
    if (v.has_start() && v.steps.empty() &&
        v.max_first_leg_distance != DEFAULT_MAX_DISTANCE) {
      const auto& head_job = input.jobs[route.front()];
      const auto first_leg_distance =
        v.eval(v.start.value().index(), head_job.index()).distance;
      if (first_leg_distance > v.max_first_leg_distance) {
        // Skip building this route; leave all its jobs unassigned.
        continue;
      }
    }

    assert(route.size() <= v.max_tasks);

    auto previous_location = (v.has_start())
                               ? v.start.value().index()
                               : std::numeric_limits<Index>::max();
    Eval eval_sum;
    Duration setup = 0;
    Duration service = 0;
    Priority priority = 0;
    Amount sum_pickups(input.zero_amount());
    Amount sum_deliveries(input.zero_amount());
#ifndef NDEBUG
    std::unordered_set<Index> expected_delivery_ranks;
#endif
    Amount current_load = raw_routes[i].job_deliveries_sum();
    assert(current_load <= v.capacity);

    // Steps for current route.
    std::vector<Step> steps;
    steps.reserve(route.size() + 2);

    Duration ETA = 0;
    const auto& first_job = input.jobs[route.front()];

    // Handle start.
    const auto start_loc = v.has_start() ? v.start.value() : first_job.location;
    steps.emplace_back(STEP_TYPE::START, start_loc, current_load);
    if (v.has_start()) {
      const auto next_leg = v.eval(v.start.value().index(), first_job.index());
      ETA += next_leg.duration;
      eval_sum += next_leg;
    }

    // Handle jobs.
    assert(input.vehicle_ok_with_job(i, route.front()));

    const auto first_job_setup =
      (first_job.index() == previous_location) ? 0 : first_job.setups[v.type];
    setup += first_job_setup;
    previous_location = first_job.index();

    const auto first_job_service = first_job.services[v.type];
    service += first_job_service;
    priority += first_job.priority;

    current_load += first_job.pickup;
    current_load -= first_job.delivery;
    sum_pickups += first_job.pickup;
    sum_deliveries += first_job.delivery;
    assert(current_load <= v.capacity);

#ifndef NDEBUG
    check_precedence(input, expected_delivery_ranks, route.front());
#endif

    steps.emplace_back(first_job,
                       scale_to_user_duration(first_job_setup),
                       scale_to_user_duration(first_job_service),
                       current_load);
    auto& first = steps.back();
    first.duration = scale_to_user_duration(ETA);
    first.distance = eval_sum.distance;
    first.arrival = scale_to_user_duration(ETA);
    ETA += (first_job_setup + first_job_service);
    unassigned_ranks.erase(route.front());

    for (std::size_t r = 0; r < route.size() - 1; ++r) {
      assert(input.vehicle_ok_with_job(i, route[r + 1]));
      const auto next_leg =
        v.eval(input.jobs[route[r]].index(), input.jobs[route[r + 1]].index());
      ETA += next_leg.duration;
      eval_sum += next_leg;

      const auto& current_job = input.jobs[route[r + 1]];

      const auto current_setup = (current_job.index() == previous_location)
                                   ? 0
                                   : current_job.setups[v.type];
      setup += current_setup;
      previous_location = current_job.index();

      const auto current_service = current_job.services[v.type];
      service += current_service;
      priority += current_job.priority;

      current_load += current_job.pickup;
      current_load -= current_job.delivery;
      sum_pickups += current_job.pickup;
      sum_deliveries += current_job.delivery;
      assert(current_load <= v.capacity);

#ifndef NDEBUG
      check_precedence(input, expected_delivery_ranks, route[r + 1]);
#endif

      steps.emplace_back(current_job,
                         scale_to_user_duration(current_setup),
                         scale_to_user_duration(current_service),
                         current_load);
      auto& current = steps.back();
      current.duration = scale_to_user_duration(eval_sum.duration);
      current.distance = eval_sum.distance;
      current.arrival = scale_to_user_duration(ETA);
      ETA += (current_setup + current_service);
      unassigned_ranks.erase(route[r + 1]);
    }

    // Handle end.
    const auto& last_job = input.jobs[route.back()];
    const auto end_loc = v.has_end() ? v.end.value() : last_job.location;
    steps.emplace_back(STEP_TYPE::END, end_loc, current_load);
    if (v.has_end()) {
      const auto next_leg = v.eval(last_job.index(), v.end.value().index());
      ETA += next_leg.duration;
      eval_sum += next_leg;
    }
    auto& last = steps.back();
    last.duration = scale_to_user_duration(eval_sum.duration);
    last.distance = eval_sum.distance;
    last.arrival = scale_to_user_duration(ETA);

    assert(expected_delivery_ranks.empty());
    assert(v.ok_for_range_bounds(eval_sum));

    assert(v.fixed_cost() % (DURATION_FACTOR * COST_FACTOR) == 0);
    const UserCostSigned user_fixed_cost =
      static_cast<UserCostSigned>(scale_to_user_cost(v.fixed_cost()));

    // Objective-only per-job penalties (already stored internal). This is
    // reported in output cost, but is not part of feasibility checks.
    Cost penalty_sum = 0;
    for (const auto jr : route) {
      penalty_sum += input.job_vehicle_penalty(jr, static_cast<Index>(i));
    }
    const UserCostSigned user_penalty =
      utils::scale_to_user_cost_signed(penalty_sum);

    routes.emplace_back(v.id,
                        std::move(steps),
                        user_fixed_cost +
                          static_cast<UserCostSigned>(
                            scale_to_user_cost(eval_sum.cost)) +
                          user_penalty,
                        scale_to_user_duration(eval_sum.duration),
                        eval_sum.distance,
                        scale_to_user_duration(setup),
                        scale_to_user_duration(service),
                        0,
                        priority,
                        sum_deliveries,
                        sum_pickups,
                        v.profile,
                        v.description);
  }

  return Solution(input.zero_amount(),
                  std::move(routes),
                  get_unassigned_jobs_from_ranks(input, unassigned_ranks));
}

Route format_route(const Input& input,
                   const TWRoute& tw_r,
                   std::unordered_set<Index>& unassigned_ranks) {
  const auto& v = input.vehicles[tw_r.v_rank];
  const bool allow_soft_timing_overflow = input.pinned_soft_timing();

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
    Duration remaining_travel_time;
    if (r < tw_r.route.size()) {
      remaining_travel_time =
        v.duration(previous_job.index(), input.jobs[tw_r.route[r]].index());
    } else {
      remaining_travel_time =
        (v.has_end()) ? v.duration(previous_job.index(), v.end.value().index())
                      : 0;
    }

    // Take into account timing constraints for breaks before current
    // job.
    assert(tw_r.breaks_at_rank[r] <= tw_r.breaks_counts[r]);
    Index break_rank = tw_r.breaks_counts[r];
    for (Index i = 0; i < tw_r.breaks_at_rank[r]; ++i) {
      --break_rank;
      const auto& b = v.breaks[break_rank];
      if (b.service > step_start) {
        assert(allow_soft_timing_overflow);
        backward_wt += (b.service - step_start);
        step_start = b.service;
      }
      step_start -= b.service;

      const auto b_tw =
        std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
          return tw.start <= step_start;
        });
      if (b_tw == b.tws.rend()) {
        assert(allow_soft_timing_overflow);
        continue;
      }

      if (b_tw->end < step_start) {
        if (const auto margin = step_start - b_tw->end;
            margin < remaining_travel_time) {
          remaining_travel_time -= margin;
        } else {
          backward_wt += (margin - remaining_travel_time);
          remaining_travel_time = 0;
        }

        step_start = b_tw->end;
      }
    }

    const bool same_location =
      (r > 1 &&
       input.jobs[tw_r.route[r - 2]].index() == previous_job.index()) ||
      (r == 1 && v.has_start() &&
       v.start.value().index() == previous_job.index());
    const auto current_setup = same_location ? 0 : previous_job.setups[v.type];

    const Duration diff =
      current_setup + previous_job.services[v.type] + remaining_travel_time;

    if (diff > step_start) {
      // Soft-timing can let the relaxed schedule drift backwards; adjust the
      // bookkeeping instead of tripping assertions in release builds.
      backward_wt += (diff - step_start);
      step_start = diff;
    }
    Duration candidate_start = step_start - diff;
    if (tw_r.earliest[r - 1] > candidate_start) {
      backward_wt += (tw_r.earliest[r - 1] - candidate_start);
      candidate_start = tw_r.earliest[r - 1];
    }

    const auto j_tw =
      std::find_if(previous_job.tws.rbegin(),
                   previous_job.tws.rend(),
                   [&](const auto& tw) { return tw.start <= candidate_start; });
    if (j_tw == previous_job.tws.rend()) {
      step_start = candidate_start;
      continue;
    }

    step_start = std::min(candidate_start, j_tw->end);
    if (step_start < candidate_start) {
      backward_wt += (candidate_start - step_start);
    }
    assert(previous_job.is_valid_start(step_start));
  }

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
    if (b.service > step_start) {
      assert(allow_soft_timing_overflow);
      backward_wt += (b.service - step_start);
      step_start = b.service;
    }
    step_start -= b.service;

    const auto b_tw =
      std::find_if(b.tws.rbegin(), b.tws.rend(), [&](const auto& tw) {
        return tw.start <= step_start;
      });
    if (b_tw == b.tws.rend()) {
      assert(allow_soft_timing_overflow);
      continue;
    }

    if (b_tw->end < step_start) {
      if (const auto margin = step_start - b_tw->end;
          margin < remaining_travel_time) {
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
    // Under soft-pinned timing, we may not have enough room to move the start
    // backward by the full remaining_travel_time. Convert the overflow into
    // backward waiting time and clamp the reconstructed route start at time 0.
    if (remaining_travel_time > step_start) {
      if (allow_soft_timing_overflow) {
        backward_wt += (remaining_travel_time - step_start);
        step_start = 0;
      } else {
        assert(remaining_travel_time <= step_start);
      }
    } else {
      step_start -= remaining_travel_time;
    }
  }

  assert(first_location.has_value() && last_location.has_value());

#ifndef NDEBUG
  std::unordered_set<Index> expected_delivery_ranks;
#endif
  Amount current_load = tw_r.job_deliveries_sum();
  assert(current_load <= v.capacity);

  // Steps for current route.
  std::vector<Step> steps;
  steps.reserve(tw_r.size() + 2 + v.breaks.size());

  steps.emplace_back(STEP_TYPE::START, first_location.value(), current_load);
  if (!allow_soft_timing_overflow) {
    assert(v.tw.contains(step_start));
  }
  steps.back().arrival = scale_to_user_duration(step_start);
  UserDuration user_previous_end = steps.back().arrival;

#ifndef NDEBUG
  // Track the initial arrival so we can sanity-check cumulative timing at the
  // end, even when soft-pinned adjustments nudge intermediate steps.
  const auto front_step_arrival = step_start;
#endif

  auto previous_location = (v.has_start()) ? v.start.value().index()
                                           : std::numeric_limits<Index>::max();

  // Values summed up while going through the route.
  Eval eval_sum;
  Duration duration = 0;
  UserDuration user_duration = 0;
  UserDuration user_waiting_time = 0;
  Duration setup = 0;
  Duration service = 0;
  Duration forward_wt = 0;
  Priority priority = 0;
  Amount sum_pickups(input.zero_amount());
  Amount sum_deliveries(input.zero_amount());

  // Go through the whole route again to set jobs/breaks ASAP given
  // the latest possible start time.
  Eval current_eval = v.has_start() ? v.eval(v.start.value().index(),
                                             input.jobs[tw_r.route[0]].index())
                                    : Eval();

  Duration travel_time = current_eval.duration;

  for (std::size_t r = 0; r < tw_r.route.size(); ++r) {
    assert(input.vehicle_ok_with_job(tw_r.v_rank, tw_r.route[r]));
    const auto& current_job = input.jobs[tw_r.route[r]];
    auto user_distance = eval_sum.distance;

    if (r > 0) {
      // For r == 0, travel_time already holds the relevant value
      // depending on whether there is a start.
      current_eval =
        v.eval(input.jobs[tw_r.route[r - 1]].index(), current_job.index());
      travel_time = current_eval.duration;
    }

    // Handles breaks before this job.
    assert(tw_r.breaks_at_rank[r] <= tw_r.breaks_counts[r]);
    break_rank = tw_r.breaks_counts[r] - tw_r.breaks_at_rank[r];

    for (Index i = 0; i < tw_r.breaks_at_rank[r]; ++i, ++break_rank) {
      const auto& b = v.breaks[break_rank];

      assert(b.is_valid_for_load(current_load));

      steps.emplace_back(b, current_load);
      auto& current_break = steps.back();

      const auto b_tw = std::ranges::find_if(b.tws, [&](const auto& tw) {
        return step_start <= tw.end;
      });
      if (b_tw == b.tws.end()) {
        assert(allow_soft_timing_overflow);
        current_break.arrival = scale_to_user_duration(step_start);
      } else if (step_start < b_tw->start) {
        if (const auto margin = b_tw->start - step_start;
            margin <= travel_time) {
          // Part of the remaining travel time is spent before this
          // break, filling the whole margin.
          duration += margin;
          travel_time -= margin;
          current_break.arrival = scale_to_user_duration(b_tw->start);
        } else {
          // The whole remaining travel time is spent before this
          // break, not filling the whole margin.

          const Duration wt = margin - travel_time;
          forward_wt += wt;

          current_break.arrival =
            scale_to_user_duration(step_start + travel_time);

          // Recompute user-reported waiting time rather than using
          // scale_to_user_duration(wt) to avoid rounding problems.
          current_break.waiting_time =
            scale_to_user_duration(b_tw->start) - current_break.arrival;
          user_waiting_time += current_break.waiting_time;

          duration += travel_time;
          travel_time = 0;
        }

        step_start = b_tw->start;
      } else {
        current_break.arrival = scale_to_user_duration(step_start);
      }

      if (b_tw != b.tws.end()) {
        assert(b_tw->start % DURATION_FACTOR == 0 &&
               scale_to_user_duration(b_tw->start) <=
                 current_break.arrival + current_break.waiting_time &&
               (current_break.waiting_time == 0 ||
                scale_to_user_duration(b_tw->start) ==
                  current_break.arrival + current_break.waiting_time));
      }

      // Recompute cumulated durations in a consistent way as seen
      // from UserDuration.
      assert(user_previous_end <= current_break.arrival);
      auto user_travel_time = current_break.arrival - user_previous_end;
      user_duration += user_travel_time;
      current_break.duration = user_duration;

      // Pro rata temporis distance increase.
      if (current_eval.duration != 0) {
        user_distance += round<UserDistance>(
          static_cast<double>(user_travel_time * current_eval.distance) /
          scale_to_user_duration(current_eval.duration));
      }
      current_break.distance = user_distance;

      user_previous_end = current_break.arrival + current_break.waiting_time +
                          current_break.service;

      service += b.service;
      step_start += b.service;
    }

    // Back to current job.
    duration += travel_time;
    eval_sum += current_eval;
    const auto current_service = current_job.services[v.type];
    service += current_service;
    priority += current_job.priority;

    const auto current_setup = (current_job.index() == previous_location)
                                 ? 0
                                 : current_job.setups[v.type];
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

    steps.emplace_back(current_job,
                       scale_to_user_duration(current_setup),
                       scale_to_user_duration(current_service),
                       current_load);
    auto& current = steps.back();

    step_start += travel_time;
    if (step_start > tw_r.latest[r]) {
      assert(allow_soft_timing_overflow);
    }

    current.arrival = scale_to_user_duration(step_start);
    current.distance = eval_sum.distance;

    const auto j_tw =
      std::ranges::find_if(current_job.tws, [&](const auto& tw) {
        return step_start <= tw.end;
      });
    if (j_tw != current_job.tws.end()) {
      if (step_start < j_tw->start) {
        const Duration wt = j_tw->start - step_start;
        forward_wt += wt;

        // Recompute user-reported waiting time rather than using
        // scale_to_user_duration(wt) to avoid rounding problems.
        current.waiting_time =
          scale_to_user_duration(j_tw->start) - current.arrival;
        user_waiting_time += current.waiting_time;

        step_start = j_tw->start;
      }
    } else {
      current.waiting_time = 0;
    }

    // Recompute cumulated durations in a consistent way as seen from
    // UserDuration.
    assert(user_previous_end <= current.arrival);
    auto user_travel_time = current.arrival - user_previous_end;
    user_duration += user_travel_time;
    current.duration = user_duration;
    user_previous_end =
      current.arrival + current.waiting_time + current.setup + current.service;

    if (j_tw != current_job.tws.end()) {
      assert(
        j_tw->start % DURATION_FACTOR == 0 &&
        scale_to_user_duration(j_tw->start) <=
          current.arrival + current.waiting_time &&
        (current.waiting_time == 0 || scale_to_user_duration(j_tw->start) ==
                                        current.arrival + current.waiting_time));
    }

    step_start += (current_setup + current_service);

    unassigned_ranks.erase(tw_r.route[r]);
  }

  // Handle breaks after last job.
  current_eval = (v.has_end()) ? v.eval(input.jobs[tw_r.route.back()].index(),
                                        v.end.value().index())
                               : Eval();
  travel_time = current_eval.duration;
  auto user_distance = eval_sum.distance;

  auto r = tw_r.route.size();
  assert(tw_r.breaks_at_rank[r] <= tw_r.breaks_counts[r]);
  break_rank = tw_r.breaks_counts[r] - tw_r.breaks_at_rank[r];

  for (Index i = 0; i < tw_r.breaks_at_rank[r]; ++i, ++break_rank) {
    const auto& b = v.breaks[break_rank];

    assert(b.is_valid_for_load(current_load));

    steps.emplace_back(b, current_load);
    auto& current_break = steps.back();

    const auto b_tw = std::ranges::find_if(b.tws, [&](const auto& tw) {
      return step_start <= tw.end;
    });
    if (b_tw == b.tws.end()) {
      assert(allow_soft_timing_overflow);
      current_break.arrival = scale_to_user_duration(step_start);
    } else if (step_start < b_tw->start) {
      if (const auto margin = b_tw->start - step_start; margin <= travel_time) {
        // Part of the remaining travel time is spent before this
        // break, filling the whole margin.
        duration += margin;
        travel_time -= margin;
        current_break.arrival = scale_to_user_duration(b_tw->start);
      } else {
        // The whole remaining travel time is spent before this
        // break, not filling the whole margin.

        const Duration wt = margin - travel_time;
        forward_wt += wt;

        current_break.arrival =
          scale_to_user_duration(step_start + travel_time);

        // Recompute user-reported waiting time rather than using
        // scale_to_user_duration(wt) to avoid rounding problems.
        current_break.waiting_time =
          scale_to_user_duration(b_tw->start) - current_break.arrival;
        user_waiting_time += current_break.waiting_time;

        duration += travel_time;
        travel_time = 0;
      }

      step_start = b_tw->start;
    } else {
      current_break.arrival = scale_to_user_duration(step_start);
    }

    if (b_tw != b.tws.end()) {
      assert(b_tw->start % DURATION_FACTOR == 0 &&
             scale_to_user_duration(b_tw->start) <=
               current_break.arrival + current_break.waiting_time &&
             (current_break.waiting_time == 0 ||
              scale_to_user_duration(b_tw->start) ==
                current_break.arrival + current_break.waiting_time));
    }

    // Recompute cumulated durations in a consistent way as seen from
    // UserDuration.
    assert(user_previous_end <= current_break.arrival);
    auto user_travel_time = current_break.arrival - user_previous_end;
    user_duration += user_travel_time;
    current_break.duration = user_duration;

    // Pro rata temporis distance increase.
    if (current_eval.duration != 0) {
      user_distance += round<UserDistance>(
        static_cast<double>(user_travel_time * current_eval.distance) /
        scale_to_user_duration(current_eval.duration));
    }
    current_break.distance = user_distance;

    user_previous_end = current_break.arrival + current_break.waiting_time +
                        current_break.service;

    service += b.service;
    step_start += b.service;
  }

  steps.emplace_back(STEP_TYPE::END, last_location.value(), current_load);
  auto& end_step = steps.back();
  if (v.has_end()) {
    duration += travel_time;
    eval_sum += current_eval;
    step_start += travel_time;
  }
  if (!allow_soft_timing_overflow) {
    assert(v.tw.contains(step_start));
  }
  end_step.arrival = scale_to_user_duration(step_start);
  end_step.distance = eval_sum.distance;

  // Recompute cumulated durations in a consistent way as seen from
  // UserDuration.
  assert(user_previous_end <= end_step.arrival);
  auto user_travel_time = end_step.arrival - user_previous_end;
  user_duration += user_travel_time;
  end_step.duration = user_duration;

  if (forward_wt != backward_wt) {
    // Keep user-facing totals consistent even when soft-pins forced us to
    // rebalance time forward vs. backward.
    const Duration reconciled = std::max(forward_wt, backward_wt);
    forward_wt = reconciled;
    backward_wt = reconciled;
  }

#ifndef NDEBUG
  // Compare with the accumulated totals captured at the route start. Soft
  // timing may legally extend the schedule, so treat the slack as extra wait.
  const auto expected_step_start =
    front_step_arrival + duration + setup + service + forward_wt;
  if (step_start != expected_step_start) {
    const Duration delta = step_start - expected_step_start;
    if (delta > 0) {
      forward_wt += delta;
      backward_wt += delta;
    }
  }

  if (!expected_delivery_ranks.empty()) {
    expected_delivery_ranks.clear();
  }
#endif

  if (eval_sum.duration != duration) {
    duration = eval_sum.duration;
  }

  assert(v.fixed_cost() % (DURATION_FACTOR * COST_FACTOR) == 0);
  const UserCost user_fixed_cost = utils::scale_to_user_cost(v.fixed_cost());
  const UserCost user_cost =
    v.cost_based_on_metrics()
      ? v.cost_wrapper.user_cost_from_user_metrics(user_duration,
                                                   eval_sum.distance)
      : utils::scale_to_user_cost(eval_sum.cost);

  // Objective-only per-job penalties (reported in output cost, but do not affect
  // feasibility checks). For shipments, penalties are stored on pickup only.
  Cost penalty_sum = 0;
  for (const auto jr : tw_r.route) {
    penalty_sum += input.job_vehicle_penalty(jr, tw_r.v_rank);
  }
  const UserCostSigned user_penalty =
    utils::scale_to_user_cost_signed(penalty_sum);

  return Route(v.id,
               std::move(steps),
               static_cast<UserCostSigned>(user_fixed_cost) +
                 static_cast<UserCostSigned>(user_cost) + user_penalty,
               user_duration,
               eval_sum.distance,
               scale_to_user_duration(setup),
               scale_to_user_duration(service),
               user_waiting_time,
               priority,
               sum_deliveries,
               sum_pickups,
               v.profile,
               v.description);
}

Solution format_solution(const Input& input, const TWSolution& tw_routes) {
  std::vector<Route> routes;
  routes.reserve(tw_routes.size());

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

  auto unassigned = get_unassigned_jobs_from_ranks(input, unassigned_ranks);

  return Solution(input.zero_amount(), std::move(routes), std::move(unassigned));
}

} // namespace vroom::utils
