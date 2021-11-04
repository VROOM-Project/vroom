/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <cmath>
#include <numeric>

#include <glpk.h>

#include "algorithms/validation/choose_ETA.h"

namespace vroom {
namespace validation {

inline Duration get_duration(double d) {
  return static_cast<Duration>(std::round(d));
}

inline Duration get_violation(const std::vector<TimeWindow>& tws,
                              Duration arrival) {
  Duration violation = 0;
  const auto tw = std::find_if(tws.begin(), tws.end(), [&](const auto& tw) {
    return arrival <= tw.end;
  });
  if (tw == tws.end()) {
    // Delay from last time window.
    violation = (arrival - tws.back().end);
  } else {
    // No violation if arrival in this time window (tw->start <=
    // arrival).
    if (arrival < tw->start) {
      if (tw == tws.begin()) {
        // No previous time window, so it's a lead time.
        violation = (tw->start - arrival);
      } else {
        // Pick smallest violation between both time windows.
        const auto previous_tw = std::prev(tw, 1);
        violation = std::min(arrival - previous_tw->end, tw->start - arrival);
      }
    }
  }

  return violation;
}

Route choose_ETA(const Input& input,
                 unsigned vehicle_rank,
                 const std::vector<VehicleStep>& steps) {
  const auto& v = input.vehicles[vehicle_rank];

  // Number of tasks except start and end.
  assert(2 < steps.size());
  const unsigned n = steps.size() - 2;

  // Total number of time windows.
  unsigned K = 0;

  // For 0 <= i <= n, if i is in J at rank r (i.e. T_i is a non-break
  // task), then B[r] is the number of tasks following T_i that are
  // breaks, durations[r] is the travel duration from task T_i to the
  // next non-break task and action_time[r] is the action time
  // (service or setup + service) for job i. Note: when vehicle has no
  // start, T_0 is a "ghost" step.
  std::vector<unsigned> J;
  std::vector<unsigned> B;
  std::vector<Duration> durations;
  std::vector<Duration> action_times;
  // Lower bound for timestamps in input in order to scale the MIP
  // matrix values.
  Duration horizon_start = std::numeric_limits<Duration>::max();
  Duration horizon_end = 0;
  if (!v.tw.is_default()) {
    horizon_start = std::min(horizon_start, v.tw.start);
    horizon_end = std::max(horizon_end, v.tw.end);
  }

  // Route indicators. relative_ETA stores steps ETA relative to a
  // start at 0 taking only travel and action times into account (no
  // waiting).
  Duration action_sum = 0;
  Duration duration_sum = 0;
  Cost cost_sum = 0;
  unsigned default_job_tw = 0;
  Duration relative_arrival = 0;
  std::vector<Duration> relative_ETA;
  std::optional<Index> previous_index;
  std::optional<Location> first_location;
  std::optional<Location> last_location;

  Index i = 0;
  for (const auto& step : steps) {
    switch (step.type) {
    case STEP_TYPE::START:
      if (v.has_start()) {
        previous_index = v.start.value().index();
        first_location = v.start.value();
        last_location = v.start.value();
      }
      J.push_back(i);
      B.push_back(0);
      action_times.push_back(0);
      relative_ETA.push_back(0);
      ++i;
      break;
    case STEP_TYPE::JOB: {
      const auto& job = input.jobs[step.rank];
      K += job.tws.size();

      J.push_back(i);
      B.push_back(0);

      if (job.tws.front().is_default()) {
        ++default_job_tw;
      } else {
        horizon_start = std::min(horizon_start, job.tws.front().start);
        horizon_end = std::max(horizon_end, job.tws.back().end);
      }

      // Only case where previous_index is not set is for first
      // duration in case vehicle has no start.
      assert(previous_index.has_value() or
             (durations.empty() and !v.has_start()));

      const auto current_duration =
        (previous_index.has_value())
          ? v.duration(previous_index.value(), job.index())
          : 0;
      durations.push_back(current_duration);
      duration_sum += current_duration;

      cost_sum += (previous_index.has_value())
                    ? v.cost(previous_index.value(), job.index())
                    : 0;

      relative_arrival += current_duration;
      relative_ETA.push_back(relative_arrival);

      const bool has_setup_time =
        !previous_index.has_value() or (previous_index.value() != job.index());
      const auto current_action =
        has_setup_time ? job.setup + job.service : job.service;
      action_times.push_back(current_action);
      action_sum += current_action;
      relative_arrival += current_action;

      previous_index = job.index();
      if (!first_location.has_value()) {
        first_location = job.location;
      }
      last_location = job.location;
      ++i;
      break;
    }
    case STEP_TYPE::BREAK: {
      const auto& b = v.breaks[step.rank];
      K += b.tws.size();

      ++B.back();
      ++i;

      action_sum += b.service;
      if (!b.tws.front().is_default()) {
        horizon_start = std::min(horizon_start, b.tws.front().start);
        horizon_end = std::max(horizon_end, b.tws.back().end);
      }

      relative_ETA.push_back(relative_arrival);
      relative_arrival += b.service;
      break;
    }
    case STEP_TYPE::END:
      if (v.has_end()) {
        assert(previous_index.has_value());
        const auto& current_duration =
          v.duration(previous_index.value(), v.end.value().index());
        durations.push_back(current_duration);
        duration_sum += current_duration;
        relative_arrival += current_duration;

        cost_sum += v.cost(previous_index.value(), v.end.value().index());

        if (!first_location.has_value()) {
          first_location = v.end.value();
        }
        last_location = v.end.value();
      } else {
        durations.push_back(0);
      }

      relative_ETA.push_back(relative_arrival);
      break;
    }
  }
  assert(first_location.has_value() and last_location.has_value());
  assert(i == n + 1);
  assert(relative_ETA.size() == steps.size());

  // Determine earliest possible start based on "service_at" and
  // "service_before" constraints.
  std::vector<Duration> latest_dates(steps.size(),
                                     std::numeric_limits<Duration>::max());
  auto start_candidate = std::numeric_limits<Duration>::max();
  for (unsigned s = 0; s < steps.size(); ++s) {
    const auto& step = steps[s];

    auto& latest_date = latest_dates[s];
    if (step.forced_service.at.has_value()) {
      latest_date = step.forced_service.at.value();
    }
    if (step.forced_service.before.has_value()) {
      latest_date = std::min(latest_date, step.forced_service.before.value());
    }
    if (latest_date != std::numeric_limits<Duration>::max()) {
      const auto reach_time = relative_ETA[s];
      if (latest_date < reach_time) {
        throw Exception(ERROR::INPUT,
                        "Infeasible route for vehicle " + std::to_string(v.id) +
                          ".");
      }
      start_candidate = std::min(start_candidate, latest_date - reach_time);
    }
  }

  // Generate a sample solution yielding an upper bound for the sum of
  // violations.
  if (!v.tw.is_default()) {
    // Start ASAP for vehicle with custom time window.
    start_candidate = std::min(start_candidate, v.tw.start);
  } else {
    if (horizon_start == std::numeric_limits<Duration>::max()) {
      // No real time window in problem input.
      start_candidate = 0;
    } else {
      // Start ASAP based on other time windows.
      start_candidate = std::min(start_candidate, horizon_start);
    }
  }
  Duration sample_violations = 0;
  // Store margin between current horizon start (resp. end) and first
  // availability date (resp. deadline). step_has_TW will help down
  // the line to decide whether going past current horizon actually
  // incurs a violation or not.
  std::vector<Duration> horizon_start_lead_times(steps.size(), 0);
  std::vector<Duration> horizon_end_delays(steps.size(), 0);
  std::vector<bool> step_has_TW(steps.size(), false);
  auto earliest_date = start_candidate;
  for (unsigned s = 0; s < steps.size(); ++s) {
    const auto& step = steps[s];
    if (s > 0) {
      earliest_date += (relative_ETA[s] - relative_ETA[s - 1]);
    }
    if (step.forced_service.at.has_value()) {
      earliest_date = std::max(earliest_date, step.forced_service.at.value());
    }
    if (step.forced_service.after.has_value()) {
      earliest_date =
        std::max(earliest_date, step.forced_service.after.value());
    }
    if (earliest_date > latest_dates[s]) {
      throw Exception(ERROR::INPUT,
                      "Infeasible route for vehicle " + std::to_string(v.id) +
                        ".");
    }

    switch (step.type) {
    case STEP_TYPE::START:
      if (!v.tw.is_default()) {
        step_has_TW[s] = true;

        if (earliest_date < v.tw.start) {
          sample_violations += (v.tw.start - earliest_date);
        }
        horizon_start_lead_times[s] = v.tw.start - horizon_start;
      }
      break;
    case STEP_TYPE::JOB: {
      sample_violations +=
        get_violation(input.jobs[step.rank].tws, earliest_date);

      const auto& tws = input.jobs[step.rank].tws;
      if ((tws.size() != 1) or !tws.front().is_default()) {
        step_has_TW[s] = true;

        horizon_start_lead_times[s] = tws.front().start - horizon_start;
        horizon_end_delays[s] = horizon_end - tws.back().end;
      }
      break;
    }
    case STEP_TYPE::BREAK: {
      sample_violations +=
        get_violation(v.breaks[step.rank].tws, earliest_date);

      const auto& tws = v.breaks[step.rank].tws;
      if ((tws.size() != 1) or !tws.front().is_default()) {
        step_has_TW[s] = true;

        horizon_start_lead_times[s] = tws.front().start - horizon_start;
        horizon_end_delays[s] = horizon_end - tws.back().end;
      }
      break;
    }
    case STEP_TYPE::END:
      if (!v.tw.is_default()) {
        step_has_TW[s] = true;

        if (v.tw.end < earliest_date) {
          sample_violations += (earliest_date - v.tw.end);
        }
        horizon_end_delays[s] = horizon_end - v.tw.end;
      }
      break;
    }
  }

  // Refine planning horizon.
  auto makespan_estimate = duration_sum + action_sum;

  if (horizon_start == std::numeric_limits<Duration>::max()) {
    // No real time window in problem input, planning horizon will
    // start at 0.
    assert(horizon_end == 0);
    horizon_start = 0;
    horizon_end = std::numeric_limits<Duration>::max();
  } else {
    // Advance "absolute" planning horizon start so as to allow lead
    // time at startup. Compute minimal delay values for possible start of
    // steps at horizon_end. See when it goes over our total
    // violations upper bound in order to reduce margin used after
    // horizon_end.
    Duration horizon_start_margin = 0;
    for (unsigned s = 0; s < steps.size(); ++s) {
      // Compute minimal delay value when step at rank s happens
      // exactly at horizon_start.
      if (relative_ETA[s] > horizon_start) {
        // Not that much margin for horizon start anyway, no point in
        // not starting at 0.
        horizon_start_margin = 0;
        break;
      }
      horizon_start_margin = relative_ETA[s];
      Duration minimal_lead_time = 0;
      for (unsigned t = 0; t <= s; ++t) {
        minimal_lead_time += horizon_start_lead_times[t];
        if (step_has_TW[t]) {
          minimal_lead_time += relative_ETA[s] - relative_ETA[t];
        }
      }

      if (minimal_lead_time > sample_violations) {
        break;
      }
    }
    assert(horizon_start_margin <= horizon_start);
    horizon_start -= horizon_start_margin;

    // Push "absolute" planning horizon end so as to allow
    // delays. Compute minimal delay values for possible start of
    // steps at horizon_end. See when it goes over our total
    // violations upper bound in order to reduce margin used after
    // horizon_end.
    Duration horizon_end_margin = 0;
    for (unsigned s = 0; s < steps.size(); ++s) {
      const auto rev_s = steps.size() - 1 - s;
      // Compute minimal delay value when step at rank rev_s happens
      // exactly at horizon_end.
      horizon_end_margin = relative_ETA.back() - relative_ETA[rev_s];
      Duration minimal_delay = 0;
      for (unsigned t = rev_s; t < steps.size(); ++t) {
        minimal_delay += horizon_end_delays[t];
        if (step_has_TW[t]) {
          minimal_delay += relative_ETA[t] - relative_ETA[rev_s];
        }
      }

      if (minimal_delay > sample_violations) {
        break;
      }
    }
    horizon_end += horizon_end_margin;

    if (makespan_estimate == 0) {
      makespan_estimate = horizon_end - horizon_start;
    }
  }

  // Retrieve user-provided upper bounds for t_i values. Retrieve
  // user-provided lower bounds for t_i values while propagating
  // travel/action constraints. Along the way, we store the rank of
  // the first relevant TW (used below to force some binary variables
  // to zero).
  std::vector<Duration> t_i_LB;
  std::vector<Duration> t_i_UB;
  Duration previous_LB = horizon_start;
  Duration previous_action = 0;
  Duration previous_travel = durations.front();
  std::vector<unsigned> first_relevant_tw_rank;
  Index rank_in_J = 0;

  for (const auto& step : steps) {
    // Derive basic bounds from user input.
    Duration LB = horizon_start;
    Duration UB = horizon_end;
    if (step.forced_service.at.has_value()) {
      const auto forced_at = step.forced_service.at.value();
      horizon_start = std::min(horizon_start, forced_at);
      horizon_end = std::max(horizon_end, forced_at);
      LB = forced_at;
      UB = forced_at;
    }
    if (step.forced_service.after.has_value()) {
      const auto forced_after = step.forced_service.after.value();
      horizon_start = std::min(horizon_start, forced_after);
      horizon_end = std::max(horizon_end, forced_after);
      LB = forced_after;
    }
    if (step.forced_service.before.has_value()) {
      const auto forced_before = step.forced_service.before.value();
      horizon_start = std::min(horizon_start, forced_before);
      horizon_end = std::max(horizon_end, forced_before);
      UB = forced_before;
    }

    // Now propagate some timing constraints for tighter lower bounds.
    switch (step.type) {
    case STEP_TYPE::START:
      previous_LB = LB;
      ++rank_in_J;
      break;
    case STEP_TYPE::JOB: {
      LB = std::max(LB, previous_LB + previous_action + previous_travel);
      previous_LB = LB;
      previous_action = action_times[rank_in_J];
      previous_travel = durations[rank_in_J];
      ++rank_in_J;
      break;
    }
    case STEP_TYPE::BREAK: {
      LB = std::max(LB, previous_LB + previous_action);
      previous_LB = LB;
      previous_action = v.breaks[step.rank].service;
      break;
    }
    case STEP_TYPE::END:
      LB = std::max(LB, previous_LB + previous_action + previous_travel);
      break;
    }
    t_i_LB.push_back(LB);
    t_i_UB.push_back(UB);

    if (step.type == STEP_TYPE::JOB or step.type == STEP_TYPE::BREAK) {
      // Determine rank of the first relevant TW.
      const auto& tws = (step.type == STEP_TYPE::JOB)
                          ? input.jobs[step.rank].tws
                          : v.breaks[step.rank].tws;
      unsigned tw_rank = 0;
      const auto tw =
        std::find_if(tws.rbegin(), tws.rend(), [&](const auto& tw) {
          return tw.start <= LB;
        });
      if (tw != tws.rend()) {
        tw_rank = std::distance(tw, tws.rend()) - 1;

        if (tw->end < LB and tw != tws.rbegin()) {
          // Lower bound is between two time windows.
          const auto next_tw = std::prev(tw, 1);
          if ((next_tw->start - LB) < (LB - tw->end)) {
            // Lead time to next time window will always be cheaper
            // than delay from the current one, which can be
            // discarded.
            ++tw_rank;
          }
        }
      }
      first_relevant_tw_rank.push_back(tw_rank);
    }
  }
  assert(first_relevant_tw_rank.size() == n);
  assert(rank_in_J == J.size());
  assert(t_i_LB.size() == steps.size());
  assert(t_i_UB.size() == steps.size());

  // Backward propagation for upper bounds based on travel/action
  // constraints. Along the way, we store the rank of the last
  // relevant TW (used below to force some binary variables to zero).
  std::vector<unsigned> last_relevant_tw_rank(n);
  Duration next_UB = t_i_UB.back();
  Duration break_travel_margin = 0;
  for (unsigned i = 0; i < steps.size(); ++i) {
    auto step_rank = steps.size() - 1 - i;
    const auto& step = steps[step_rank];

    switch (step.type) {
    case STEP_TYPE::START:
      assert(rank_in_J == 1);
      t_i_UB[step_rank] = std::min(t_i_UB[step_rank], next_UB - durations[0]);
      break;
    case STEP_TYPE::JOB: {
      --rank_in_J;
      const auto action = action_times[rank_in_J];
      const auto next_travel = (break_travel_margin < durations[rank_in_J])
                                 ? durations[rank_in_J] - break_travel_margin
                                 : 0;
      assert(action + next_travel <= next_UB);
      t_i_UB[step_rank] =
        std::min(t_i_UB[step_rank], next_UB - next_travel - action);
      next_UB = t_i_UB[step_rank];
      break_travel_margin = 0;
      break;
    }
    case STEP_TYPE::BREAK: {
      const auto service = v.breaks[step.rank].service;
      assert(service <= next_UB);
      const auto candidate = next_UB - service;
      if (t_i_UB[step_rank] < candidate) {
        // User-provided constraints gives margin for travel after
        // this break.
        break_travel_margin += (candidate - t_i_UB[step_rank]);
      } else {
        t_i_UB[step_rank] = candidate;
      }
      next_UB = t_i_UB[step_rank];
      break;
    }
    case STEP_TYPE::END:
      break;
    }

    if (step.type == STEP_TYPE::JOB or step.type == STEP_TYPE::BREAK) {
      // Determine rank of the last relevant TW.
      const auto UB = t_i_UB[step_rank];
      const auto& tws = (step.type == STEP_TYPE::JOB)
                          ? input.jobs[step.rank].tws
                          : v.breaks[step.rank].tws;
      unsigned tw_rank = tws.size() - 1;
      const auto tw = std::find_if(tws.begin(), tws.end(), [&](const auto& tw) {
        return UB <= tw.end;
      });
      if (tw != tws.end()) {
        tw_rank -= (std::distance(tw, tws.end()) - 1);

        if (UB < tw->start and tw != tws.begin()) {
          // Lower bound is between two time windows.
          auto prev_tw = std::prev(tw, 1);
          if ((UB - prev_tw->end) < (tw->start - UB)) {
            // Delay from the previous time window will always be
            // cheaper than lead time to the current one, which can be
            // discarded.
            --tw_rank;
          }
        }
      }
      last_relevant_tw_rank[step_rank - 1] = tw_rank;
    }
  }

  const unsigned nb_delta_constraints = J.size();
  assert(B.size() == nb_delta_constraints);
  assert(durations.size() == nb_delta_constraints);

  // 1. create problem.
  glp_prob* lp;
  lp = glp_create_prob();
  glp_set_prob_name(lp, "choose_ETA");
  glp_set_obj_dir(lp, GLP_MIN);

  // Constants and column indices.
  const unsigned nb_constraints = 4 * n + 3 + nb_delta_constraints + 2;
  const unsigned nb_non_zero =
    2 * (3 * n + 3) + 3 * K + 2 * n + 2 - default_job_tw + 2 + n + 2;
  const unsigned start_Y_col = n + 3;
  const unsigned start_X_col = 2 * n + 4 + 1;
  const unsigned start_delta_col = start_X_col + K;
  const unsigned nb_var = start_delta_col + n;

  // Set objective for first optimization round (violations and
  // makespan).
  glp_add_cols(lp, nb_var);
  for (unsigned i = 0; i <= n + 1; ++i) {
    glp_set_obj_coef(lp, start_Y_col + i, makespan_estimate);
  }
  glp_set_obj_coef(lp, n + 2, 1);
  glp_set_obj_coef(lp, 1, -1);

  // 2. handle constraints.
  glp_add_rows(lp, nb_constraints);

  unsigned current_row = 1;

  // Precedence constraints.
  glp_set_row_name(lp, current_row, "P0");
  glp_set_row_bnds(lp, current_row, GLP_LO, 0.0, 0.0);
  ++current_row;

  rank_in_J = 1;
  for (unsigned i = 0; i < n; ++i) {
    auto name = "P" + std::to_string(i + 1);
    glp_set_row_name(lp, current_row, name.c_str());
    double action;
    const auto& step = steps[1 + i];
    if (step.type == STEP_TYPE::JOB) {
      action = action_times[rank_in_J];
      ++rank_in_J;
    } else {
      assert(step.type == STEP_TYPE::BREAK);
      action = v.breaks[step.rank].service;
    }
    glp_set_row_bnds(lp, current_row, GLP_LO, action, 0.0);
    ++current_row;
  }
  assert(rank_in_J == J.size());
  assert(current_row == n + 2);

  // Vehicle TW start violation constraint.
  glp_set_row_name(lp, current_row, "L0");
  const double start_LB = v.tw.is_default() ? 0 : v.tw.start - horizon_start;
  glp_set_row_bnds(lp, current_row, GLP_LO, start_LB, 0.0);
  ++current_row;

  // Lead time ("earliest violation") constraints.
  for (unsigned i = 0; i < n; ++i) {
    auto name = "L" + std::to_string(i + 1);
    glp_set_row_name(lp, current_row, name.c_str());
    glp_set_row_bnds(lp, current_row, GLP_LO, 0.0, 0.0);
    ++current_row;
  }
  assert(current_row == 2 * n + 3);

  // Delay ("latest violation") constraints.
  for (unsigned i = 0; i < n; ++i) {
    auto name = "D" + std::to_string(i + 1);
    glp_set_row_name(lp, current_row, name.c_str());
    glp_set_row_bnds(lp, current_row, GLP_UP, 0.0, 0.0);
    ++current_row;
  }

  // Vehicle TW end violation constraint.
  auto name = "D" + std::to_string(n + 1);
  glp_set_row_name(lp, current_row, name.c_str());
  // Using v.tw.end is fine too for a default time window.
  glp_set_row_bnds(lp, current_row, GLP_UP, 0.0, v.tw.end - horizon_start);
  ++current_row;

  assert(current_row == 3 * n + 4);

  // Binary variable decision constraints.
  for (unsigned i = 1; i <= n; ++i) {
    auto name = "S" + std::to_string(i);
    glp_set_row_name(lp, current_row, name.c_str());
    glp_set_row_bnds(lp, current_row, GLP_FX, 1.0, 1.0);
    ++current_row;
  }
  assert(current_row == 4 * n + 4);

  // Delta constraints.
  for (unsigned r = 0; r < J.size(); ++r) {
    auto name = "Delta" + std::to_string(J[r]);
    glp_set_row_name(lp, current_row, name.c_str());
    glp_set_row_bnds(lp, current_row, GLP_FX, durations[r], durations[r]);
    ++current_row;
  }

  // Makespan and \sum Y_i dummy constraints (used for second solving
  // phase).
  name = "Makespan";
  glp_set_row_name(lp, current_row, name.c_str());
  glp_set_row_bnds(lp, current_row, GLP_LO, 0, 0);

  ++current_row;
  assert(current_row == nb_constraints);

  name = "Sigma_Y";
  glp_set_row_name(lp, current_row, name.c_str());
  if (sample_violations == 0) {
    glp_set_row_bnds(lp, current_row, GLP_FX, 0, 0);
  } else {
    glp_set_row_bnds(lp, current_row, GLP_DB, 0, sample_violations);
  }

  // 3. set variables and coefficients.
  unsigned current_col = 1;
  // Variables for time of services (t_i values).
  for (unsigned i = 0; i <= n + 1; ++i) {
    auto name = "t" + std::to_string(i);
    glp_set_col_name(lp, current_col, name.c_str());

    const Duration LB = t_i_LB[i];
    const Duration UB = t_i_UB[i];

    if (UB < LB) {
      throw Exception(ERROR::INPUT,
                      "Infeasible route for vehicle " + std::to_string(v.id) +
                        ".");
    }

    if (LB == UB) {
      // Fixed t_i value.
      double service_at = static_cast<double>(LB - horizon_start);
      glp_set_col_bnds(lp, current_col, GLP_FX, service_at, service_at);
    } else {
      // t_i value has a lower bound, either 0 or user-defined.
      double service_after = static_cast<double>(LB - horizon_start);
      double service_before = static_cast<double>(UB - horizon_start);
      glp_set_col_bnds(lp, current_col, GLP_DB, service_after, service_before);
    }
    ++current_col;
  }
  assert(current_col == start_Y_col);

  // Define variables for measure of TW violation.
  for (unsigned i = 0; i <= n + 1; ++i) {
    auto name = "Y" + std::to_string(i);
    glp_set_col_name(lp, current_col, name.c_str());
    glp_set_col_bnds(lp, current_col, GLP_LO, 0.0, 0.0);
    ++current_col;
  }
  assert(current_col == 2 * n + 5);

  // Binary variables for job time window choice.
  for (unsigned i = 0; i < n; ++i) {
    const auto& step = steps[1 + i];
    const auto& tws = (step.type == STEP_TYPE::JOB) ? input.jobs[step.rank].tws
                                                    : v.breaks[step.rank].tws;
    for (unsigned k = 0; k < tws.size(); ++k) {
      auto name = "X" + std::to_string(i + 1) + "_" + std::to_string(k);
      glp_set_col_name(lp, current_col, name.c_str());
      glp_set_col_kind(lp, current_col, GLP_BV);
      if (k < first_relevant_tw_rank[i] or k > last_relevant_tw_rank[i]) {
        glp_set_col_bnds(lp, current_col, GLP_FX, 0, 0);
      }
      ++current_col;
    }
  }
  assert(current_col == start_delta_col);

  // Delta variables.
  for (unsigned i = 0; i <= n; ++i) {
    auto name = "delta" + std::to_string(i);
    glp_set_col_name(lp, current_col, name.c_str());
    glp_set_col_bnds(lp, current_col, GLP_LO, 0.0, 0.0);
    ++current_col;
  }
  assert(current_col == nb_var + 1);

  // Define non-zero elements in matrix.
  int* ia = new int[1 + nb_non_zero];
  int* ja = new int[1 + nb_non_zero];
  double* ar = new double[1 + nb_non_zero];

  unsigned r = 1;
  // Coefficients for precedence constraints.
  for (unsigned i = 1; i <= n + 1; ++i) {
    // a[i,i] = -1
    ia[r] = i;
    ja[r] = i;
    ar[r] = -1;
    ++r;

    // a[i,i + 1] = 1
    ia[r] = i;
    ja[r] = i + 1;
    ar[r] = 1;
    ++r;

    // a[i, start_delta_col + i - 1] = -1
    ia[r] = i;
    ja[r] = start_delta_col + i - 1;
    ar[r] = -1;
    ++r;
  }

  unsigned constraint_rank = n + 2;

  // Coefficients for L0 constraint.
  // a[constraint_rank, 1] = 1
  ia[r] = constraint_rank;
  ja[r] = 1;
  ar[r] = 1;
  ++r;

  // a[constraint_rank, start_Y_col] = 1
  ia[r] = constraint_rank;
  ja[r] = start_Y_col;
  ar[r] = 1;
  ++r;
  ++constraint_rank;

  // Coefficients for other L_i constraints. current_X_rank is the
  // rank for binaries that describe the time window choices.
  unsigned current_X_rank = start_X_col;

  for (unsigned i = 0; i < n; ++i) {
    // a[constraint_rank, i + 2] = 1
    ia[r] = constraint_rank;
    ja[r] = i + 2;
    ar[r] = 1;
    ++r;

    // a[constraint_rank, n + 4 + i] = 1
    ia[r] = constraint_rank;
    ja[r] = n + 4 + i;
    ar[r] = 1;
    ++r;

    const auto& step = steps[1 + i];
    const auto& tws = (step.type == STEP_TYPE::JOB) ? input.jobs[step.rank].tws
                                                    : v.breaks[step.rank].tws;
    if (step.type == STEP_TYPE::JOB and tws.front().is_default()) {
      // Not setting a value in this case means the constraint will
      // always be met with matching Y set to 0.
      ++current_X_rank;
    } else {
      for (const auto& tw : tws) {
        // a[constraint_rank, current_X_rank] = - earliest_date for k-th
        // TW of task.
        ia[r] = constraint_rank;
        ja[r] = current_X_rank;
        ar[r] = -static_cast<double>(tw.start - horizon_start);
        ++r;

        ++current_X_rank;
      }
    }

    ++constraint_rank;
  }
  assert(current_X_rank == start_delta_col);
  assert(constraint_rank == 2 * n + 3);

  // Coefficients for D_i constraints.
  current_X_rank = start_X_col;

  for (unsigned i = 0; i < n; ++i) {
    // a[constraint_rank, i + 2] = 1
    ia[r] = constraint_rank;
    ja[r] = i + 2;
    ar[r] = 1;
    ++r;

    // a[constraint_rank, n + 4 + i] = -1
    ia[r] = constraint_rank;
    ja[r] = n + 4 + i;
    ar[r] = -1;
    ++r;

    const auto& step = steps[1 + i];
    const auto& tws = (step.type == STEP_TYPE::JOB) ? input.jobs[step.rank].tws
                                                    : v.breaks[step.rank].tws;
    if (step.type == STEP_TYPE::JOB and tws.front().is_default()) {
      // Set a value that makes sure this constraint is automatically
      // met with matching Y value set to 0.
      ia[r] = constraint_rank;
      ja[r] = current_X_rank;
      ar[r] = -static_cast<double>(horizon_end);
      ++r;

      ++current_X_rank;
    } else {
      for (const auto& tw : tws) {
        // a[constraint_rank, current_X_rank] = - latest_date for k-th
        // TW of task.
        ia[r] = constraint_rank;
        ja[r] = current_X_rank;
        ar[r] = -static_cast<double>(tw.end - horizon_start);
        ++r;

        ++current_X_rank;
      }
    }

    ++constraint_rank;
  }
  assert(current_X_rank == 2 * n + 4 + K + 1);

  // Coefficients D_{n + 1} constraint.
  // a[constraint_rank, n + 2] = 1
  ia[r] = constraint_rank;
  ja[r] = n + 2;
  ar[r] = 1;
  ++r;

  // a[constraint_rank, 2 * n + 4] = -1
  ia[r] = constraint_rank;
  ja[r] = 2 * n + 4;
  ar[r] = -1;
  ++r;
  ++constraint_rank;

  assert(constraint_rank == 3 * n + 4);

  // Decision constraints S_i for binary variables.
  current_X_rank = start_X_col;

  for (unsigned i = 0; i < n; ++i) {
    const auto& step = steps[1 + i];
    const auto& tws = (step.type == STEP_TYPE::JOB) ? input.jobs[step.rank].tws
                                                    : v.breaks[step.rank].tws;

    for (unsigned k = 0; k < tws.size(); ++k) {
      // a[constraint_rank, current_X_rank] = 1
      ia[r] = constraint_rank;
      ja[r] = current_X_rank;
      ar[r] = 1;
      ++r;

      ++current_X_rank;
    }
    ++constraint_rank;
  }
  assert(current_X_rank == start_delta_col);
  assert(constraint_rank == 4 * n + 4);

  // Delta_i constraints. Going through all delta variables exactly
  // once using B.
  unsigned current_delta_rank = start_delta_col;
  for (const auto Bi : B) {
    const auto row_limit = current_delta_rank + 1 + Bi;

    while (current_delta_rank < row_limit) {
      // a[constraint_rank, current_delta_rank] = 1
      ia[r] = constraint_rank;
      ja[r] = current_delta_rank;
      ar[r] = 1;
      ++r;
      ++current_delta_rank;
    }
    ++constraint_rank;
  }
  assert(current_delta_rank = nb_var + 1);

  // Makespan coefficients
  // a[constraint_rank, 1] = -1
  ia[r] = constraint_rank;
  ja[r] = 1;
  ar[r] = -1;
  ++r;
  // a[constraint_rank, n + 2] = 1
  ia[r] = constraint_rank;
  ja[r] = n + 2;
  ar[r] = 1;
  ++r;

  ++constraint_rank;
  assert(constraint_rank == nb_constraints);

  // \sum Y_i
  for (unsigned i = start_Y_col; i < start_X_col; ++i) {
    // a[constraint_rank, i] = 1
    ia[r] = constraint_rank;
    ja[r] = i;
    ar[r] = 1;
    ++r;
  }
  assert(r == nb_non_zero + 1);

  glp_load_matrix(lp, nb_non_zero, ia, ja, ar);

  delete[] ia;
  delete[] ja;
  delete[] ar;

  // 4. Solve for violations and makespan.
  glp_term_out(GLP_OFF);
  glp_iocp parm;
  glp_init_iocp(&parm);
  parm.presolve = GLP_ON;
  // Adjust branching heuristic due to
  // https://lists.gnu.org/archive/html/bug-glpk/2020-11/msg00001.html
  parm.br_tech = GLP_BR_MFV;

  glp_intopt(lp, &parm);

  auto status = glp_mip_status(lp);
  if (status == GLP_UNDEF or status == GLP_NOFEAS) {
    throw Exception(ERROR::INPUT,
                    "Infeasible route for vehicle " + std::to_string(v.id) +
                      ".");
  }
  // We should not get GLP_FEAS.
  assert(status == GLP_OPT);

  // const auto v_str = std::to_string(v.id);
  // glp_write_lp(lp, NULL, ("mip_1_v_" + v_str + ".lp").c_str());
  // glp_print_mip(lp, ("mip_1_v_" + v_str + ".sol").c_str());

  // 5. Solve for earliest start dates.
  // Adjust objective.
  Duration delta_sum_majorant = 0;
  current_delta_rank = start_delta_col;
  for (unsigned i = 0; i < B.size(); ++i) {
    for (unsigned k = current_delta_rank + 1; k < current_delta_rank + 1 + B[i];
         ++k) {
      glp_set_obj_coef(lp, k, k - current_delta_rank);
    }
    current_delta_rank += (1 + B[i]);
    delta_sum_majorant += (B[i] * durations[i]);
  }
  assert(current_delta_rank = nb_var + 1);

  for (unsigned i = 0; i <= n + 1; ++i) {
    glp_set_obj_coef(lp, start_Y_col + i, 0);
  }
  glp_set_obj_coef(lp, n + 2, 0);
  glp_set_obj_coef(lp, 1, 0);

  const auto M = (delta_sum_majorant == 0) ? 1 : delta_sum_majorant;
  for (unsigned i = 2; i <= n + 1; ++i) {
    glp_set_obj_coef(lp, i, M);
  }

  // Add constraint to fix makespan.
  const Duration best_makespan = get_duration(glp_mip_col_val(lp, n + 2)) -
                                 get_duration(glp_mip_col_val(lp, 1));
  glp_set_row_bnds(lp,
                   nb_constraints - 1,
                   GLP_FX,
                   best_makespan,
                   best_makespan);
  // Pin Y_i sum.
  Duration sum_y_i = 0;
  for (unsigned i = start_Y_col; i < start_X_col; ++i) {
    sum_y_i += get_duration(glp_mip_col_val(lp, i));
  }
  glp_set_row_bnds(lp, nb_constraints, GLP_FX, sum_y_i, sum_y_i);

  glp_intopt(lp, &parm);

  // glp_write_lp(lp, NULL, ("mip_2_v_" + v_str + ".lp").c_str());
  // glp_print_mip(lp, ("mip_2_v_" + v_str + ".sol").c_str());

  status = glp_mip_status(lp);
  if (status == GLP_UNDEF or status == GLP_NOFEAS) {
    throw Exception(ERROR::INPUT,
                    "Infeasible route for vehicle " + std::to_string(v.id) +
                      ".");
  }
  // We should not get GLP_FEAS.
  assert(status == GLP_OPT);

  // Get output.
  const Duration v_start = horizon_start + get_duration(glp_mip_col_val(lp, 1));
  const Duration v_end =
    horizon_start + get_duration(glp_mip_col_val(lp, n + 2));
  const Duration start_travel =
    get_duration(glp_mip_col_val(lp, start_delta_col));

  std::vector<Duration> task_ETA;
  std::vector<Duration> task_travels;
  for (unsigned i = 0; i < n; ++i) {
    task_ETA.push_back(horizon_start +
                       get_duration(glp_mip_col_val(lp, i + 2)));
    task_travels.push_back(
      get_duration(glp_mip_col_val(lp, start_delta_col + 1 + i)));
  }

  // Populate vector storing picked time window ranks.
  current_X_rank = start_X_col;
  std::vector<Index> task_tw_ranks;

  for (const auto& step : steps) {
    switch (step.type) {
    case STEP_TYPE::START:
      break;
    case STEP_TYPE::JOB: {
      const auto& job = input.jobs[step.rank];
      for (unsigned k = 0; k < job.tws.size(); ++k) {
        auto val = get_duration(glp_mip_col_val(lp, current_X_rank));
        if (val == 1) {
          task_tw_ranks.push_back(k);
        }

        ++current_X_rank;
      }
      break;
    }
    case STEP_TYPE::BREAK: {
      const auto& b = v.breaks[step.rank];
      for (unsigned k = 0; k < b.tws.size(); ++k) {
        auto val = get_duration(glp_mip_col_val(lp, current_X_rank));
        if (val == 1) {
          task_tw_ranks.push_back(k);
        }

        ++current_X_rank;
      }
    }
    case STEP_TYPE::END:
      break;
    }
  }
  assert(current_X_rank == start_delta_col);
  assert(task_tw_ranks.size() == n);

  // Generate route.
  Duration duration = 0;
  Duration setup = 0;
  Duration service = 0;
  Duration forward_wt = 0;
  Priority priority = 0;
  Amount sum_pickups(input.zero_amount());
  Amount sum_deliveries(input.zero_amount());
  Duration lead_time = 0;
  Duration delay = 0;
  unsigned number_of_tasks = 0;
  std::unordered_set<VIOLATION> v_types;

  // Startup load is the sum of deliveries for (single) jobs.
  Amount current_load(input.zero_amount());
  for (const auto& step : steps) {
    if (step.type == STEP_TYPE::JOB and step.job_type == JOB_TYPE::SINGLE) {
      current_load += input.jobs[step.rank].delivery;
    }
  }

  // Used for precedence violations.
  std::unordered_set<Index> expected_delivery_ranks;
  std::unordered_set<Index> delivery_first_ranks;
  std::unordered_map<Index, Index> delivery_to_pickup_step_rank;

  // Used to spot missing breaks.
  std::unordered_set<Id> break_ids;
  std::transform(v.breaks.begin(),
                 v.breaks.end(),
                 std::inserter(break_ids, break_ids.end()),
                 [](const auto& b) { return b.id; });

  std::vector<Step> sol_steps;

  assert(v.has_start() or start_travel == 0);

  sol_steps.emplace_back(STEP_TYPE::START,
                         first_location.value(),
                         current_load);
  sol_steps.back().duration = 0;
  sol_steps.back().arrival = v_start;
  if (v_start < v.tw.start) {
    sol_steps.back().violations.types.insert(VIOLATION::LEAD_TIME);
    v_types.insert(VIOLATION::LEAD_TIME);
    Duration lt = v.tw.start - v_start;
    sol_steps.back().violations.lead_time = lt;
    lead_time += lt;
    assert(lt == get_duration(glp_mip_col_val(lp, start_Y_col)));
  }

  if (!(current_load <= v.capacity)) {
    sol_steps.back().violations.types.insert(VIOLATION::LOAD);
    v_types.insert(VIOLATION::LOAD);
  }

  Duration previous_start = v_start;
  previous_action = 0;
  previous_travel = start_travel;
  unsigned task_rank = 0;
  auto previous_location = (v.has_start()) ? v.start.value().index()
                                           : std::numeric_limits<Index>::max();

  for (const auto& step : steps) {
    switch (step.type) {
    case STEP_TYPE::START:
      continue;
      break;
    case STEP_TYPE::JOB: {
      auto job_rank = step.rank;
      const auto& job = input.jobs[job_rank];

      const auto current_setup =
        (previous_location != job.index()) ? job.setup : 0;
      previous_location = job.index();

      setup += current_setup;
      service += job.service;
      priority += job.priority;

      current_load += job.pickup;
      current_load -= job.delivery;
      sum_pickups += job.pickup;
      sum_deliveries += job.delivery;

      sol_steps.emplace_back(job, current_setup, current_load);
      auto& current = sol_steps.back();

      duration += previous_travel;
      current.duration = duration;

      const auto arrival = previous_start + previous_action + previous_travel;
      const auto service_start = task_ETA[task_rank];
      assert(arrival <= service_start);

      current.arrival = arrival;
      Duration wt = service_start - arrival;
      current.waiting_time = wt;
      forward_wt += wt;

      // Handle violations.
      auto tw_rank = task_tw_ranks[task_rank];
      if (service_start < job.tws[tw_rank].start) {
        current.violations.types.insert(VIOLATION::LEAD_TIME);
        v_types.insert(VIOLATION::LEAD_TIME);
        Duration lt = job.tws[tw_rank].start - service_start;
        current.violations.lead_time = lt;
        lead_time += lt;
      }
      if (job.tws[tw_rank].end < service_start) {
        current.violations.types.insert(VIOLATION::DELAY);
        v_types.insert(VIOLATION::DELAY);
        Duration dl = service_start - job.tws[tw_rank].end;
        current.violations.delay = dl;
        delay += dl;
      }
      if (!(current_load <= v.capacity)) {
        current.violations.types.insert(VIOLATION::LOAD);
        v_types.insert(VIOLATION::LOAD);
      }
      if (!input.vehicle_ok_with_job(vehicle_rank, job_rank)) {
        current.violations.types.insert(VIOLATION::SKILLS);
        v_types.insert(VIOLATION::SKILLS);
      }
      ++number_of_tasks;
      if (v.max_tasks < number_of_tasks) {
        current.violations.types.insert(VIOLATION::MAX_TASKS);
        v_types.insert(VIOLATION::MAX_TASKS);
      }

      switch (job.type) {
      case JOB_TYPE::SINGLE:
        break;
      case JOB_TYPE::PICKUP:
        if (delivery_first_ranks.find(job_rank + 1) !=
            delivery_first_ranks.end()) {
          current.violations.types.insert(VIOLATION::PRECEDENCE);
          v_types.insert(VIOLATION::PRECEDENCE);
        } else {
          expected_delivery_ranks.insert(job_rank + 1);
          delivery_to_pickup_step_rank.emplace(job_rank + 1,
                                               sol_steps.size() - 1);
        }
        break;
      case JOB_TYPE::DELIVERY:
        auto search = expected_delivery_ranks.find(job_rank);
        if (search == expected_delivery_ranks.end()) {
          current.violations.types.insert(VIOLATION::PRECEDENCE);
          v_types.insert(VIOLATION::PRECEDENCE);
          delivery_first_ranks.insert(job_rank);
        } else {
          expected_delivery_ranks.erase(search);
        }
        break;
      }

      previous_start = service_start;
      previous_action = current_setup + job.service;
      previous_travel = task_travels[task_rank];
      ++task_rank;
      break;
    }
    case STEP_TYPE::BREAK: {
      auto break_rank = step.rank;
      const auto& b = v.breaks[break_rank];

      assert(break_ids.find(b.id) != break_ids.end());
      break_ids.erase(b.id);

      service += b.service;

      sol_steps.emplace_back(b, current_load);
      auto& current = sol_steps.back();

      duration += previous_travel;
      current.duration = duration;

      const auto arrival = previous_start + previous_action + previous_travel;
      const auto service_start = task_ETA[task_rank];
      assert(arrival <= service_start);

      current.arrival = arrival;
      Duration wt = service_start - arrival;
      current.waiting_time = wt;
      forward_wt += wt;

      // Handle violations.
      auto tw_rank = task_tw_ranks[task_rank];
      if (service_start < b.tws[tw_rank].start) {
        current.violations.types.insert(VIOLATION::LEAD_TIME);
        v_types.insert(VIOLATION::LEAD_TIME);
        Duration lt = b.tws[tw_rank].start - service_start;
        current.violations.lead_time = lt;
        lead_time += lt;
      }
      if (b.tws[tw_rank].end < service_start) {
        current.violations.types.insert(VIOLATION::DELAY);
        v_types.insert(VIOLATION::DELAY);
        Duration dl = service_start - b.tws[tw_rank].end;
        current.violations.delay = dl;
        delay += dl;
      }
      if (!(current_load <= v.capacity)) {
        current.violations.types.insert(VIOLATION::LOAD);
        v_types.insert(VIOLATION::LOAD);
      }

      previous_start = service_start;
      previous_action = b.service;
      previous_travel = task_travels[task_rank];
      ++task_rank;
      break;
    }
    case STEP_TYPE::END:
      duration += previous_travel;

      const auto arrival = previous_start + previous_action + previous_travel;
      assert(arrival <= v_end);

      sol_steps.emplace_back(STEP_TYPE::END,
                             last_location.value(),
                             current_load);
      sol_steps.back().duration = duration;
      sol_steps.back().arrival = arrival;
      Duration wt = v_end - arrival;
      sol_steps.back().waiting_time = wt;
      forward_wt += wt;

      if (v.tw.end < v_end) {
        sol_steps.back().violations.types.insert(VIOLATION::DELAY);
        v_types.insert(VIOLATION::DELAY);
        Duration dl = v_end - v.tw.end;
        sol_steps.back().violations.delay = dl;
        delay += dl;
      }
      if (!(current_load <= v.capacity)) {
        sol_steps.back().violations.types.insert(VIOLATION::LOAD);
        v_types.insert(VIOLATION::LOAD);
      }
      break;
    }
  }

  assert(get_duration(glp_mip_col_val(lp, 2 * n + 4)) ==
         sol_steps.back().violations.delay);

  glp_delete_prob(lp);
  glp_free_env();

  // Precedence violations for pickups without a delivery.
  for (const auto d_rank : expected_delivery_ranks) {
    auto search = delivery_to_pickup_step_rank.find(d_rank);
    assert(search != delivery_to_pickup_step_rank.end());
    sol_steps[search->second].violations.types.insert(VIOLATION::PRECEDENCE);
    v_types.insert(VIOLATION::PRECEDENCE);
  }

  if (!break_ids.empty()) {
    v_types.insert(VIOLATION::MISSING_BREAK);
  }

  return Route(v.id,
               std::move(sol_steps),
               cost_sum,
               setup,
               service,
               duration,
               forward_wt,
               priority,
               sum_deliveries,
               sum_pickups,
               v.profile,
               v.description,
               std::move(Violations(lead_time, delay, std::move(v_types))));
}

} // namespace validation
} // namespace vroom
