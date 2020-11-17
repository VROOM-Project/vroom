/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <algorithm>
#include <numeric>

#include <glpk.h>

#include "algorithms/validation/choose_invalid.h"

namespace vroom {
namespace validation {

Route choose_invalid_route(const Input& input,
                           unsigned vehicle_rank,
                           const std::vector<InputStep>& steps,
                           std::unordered_set<Index>& unassigned_ranks) {
  const auto& m = input.get_matrix();
  const auto& v = input.vehicles[vehicle_rank];

  // Number of tasks except start and end.
  const unsigned extra_steps = (v.has_start() ? 1 : 0) + (v.has_end() ? 1 : 0);
  assert(extra_steps < steps.size());
  const unsigned n = steps.size() - extra_steps;
  const unsigned first_task_rank = (v.has_start() ? 1 : 0);

  // Total number of time windows.
  unsigned K = 0;

  // For 0 <= i <= n, if i is in J (i.e. T_i is a non-break task),
  // then B[i] is the number of tasks following T_i that are breaks,
  // and durations[i] is the travel duration from task T_i to the next
  // non-break task. Note: when vehicle has no start, T_0 is a "ghost"
  // step.
  std::vector<unsigned> J({0});
  std::vector<unsigned> B({0});
  std::vector<double> durations;

  // Use max value for last_index as "unset".
  std::optional<Index> last_index;

  // Route indicators.
  Duration service_sum = 0;
  Duration duration_sum = 0;

  Index i = 1;
  for (const auto& step : steps) {
    switch (step.type) {
    case STEP_TYPE::START:
      assert(v.has_start());
      last_index = v.start.value().index();
      break;
    case STEP_TYPE::JOB: {
      const auto& job = input.jobs[step.rank];
      K += job.tws.size();

      J.push_back(i);
      B.push_back(0);

      service_sum += job.service;

      if (last_index.has_value()) {
        const auto& current_duration = m[last_index.value()][job.index()];
        durations.push_back(current_duration);
        duration_sum += current_duration;
      } else {
        // Only happens for first duration in case vehicle has no
        // start.
        assert(durations.empty() and !v.has_start());
        durations.push_back(0);
      }
      last_index = job.index();
      ++i;
      break;
    }
    case STEP_TYPE::BREAK: {
      const auto& b = v.breaks[step.rank];
      K += b.tws.size();

      ++B.back();
      ++i;

      service_sum += b.service;
      break;
    }
    case STEP_TYPE::END:
      assert(v.has_end() and last_index.has_value());
      const auto& current_duration =
        m[last_index.value()][v.end.value().index()];
      durations.push_back(current_duration);
      duration_sum += current_duration;
      break;
    }
  }
  if (!v.has_end()) {
    durations.push_back(0);
  }
  assert(i == n + 1);

  const unsigned nb_delta_constraints = J.size();
  assert(B.size() == nb_delta_constraints);
  assert(durations.size() == nb_delta_constraints);

  // Determine objective constants.
  const double makespan_estimate =
    static_cast<double>(duration_sum + service_sum);
  const double M2 = static_cast<double>(n) * makespan_estimate;
  const double M1 = M2 * makespan_estimate;

  // Create problem.
  glp_prob* lp;
  lp = glp_create_prob();
  glp_set_prob_name(lp, "choose_ETA");
  glp_set_obj_dir(lp, GLP_MIN);

  // Define constraints and remember number of non-zero values in the
  // matrix.
  const unsigned nb_constraints = 4 * n + 3 + nb_delta_constraints;
  const unsigned nb_non_zero = 2 * (3 * n + 3) + 3 * K + 2 * n + 2;

  glp_add_rows(lp, nb_constraints);

  unsigned current_row = 1;

  // Precedence constraints.
  glp_set_row_name(lp, current_row, "P0");
  glp_set_row_bnds(lp, current_row, GLP_LO, 0.0, 0.0);
  ++current_row;

  for (unsigned i = 0; i < n; ++i) {
    auto name = "P" + std::to_string(i + 1);
    glp_set_row_name(lp, current_row, name.c_str());
    double service;
    const auto& step = steps[first_task_rank + i];
    if (step.type == STEP_TYPE::JOB) {
      service = input.jobs[step.rank].service;
    } else {
      assert(step.type == STEP_TYPE::BREAK);
      service = v.breaks[step.rank].service;
    }
    glp_set_row_bnds(lp, current_row, GLP_LO, service, 0.0);
    ++current_row;
  }

  assert(current_row == n + 2);

  // Vehicle TW start violation constraint.
  double lb = v.tw.start;
  glp_set_row_name(lp, current_row, "L0");
  glp_set_row_bnds(lp, current_row, GLP_LO, lb, 0.0);
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
  glp_set_row_bnds(lp, current_row, GLP_UP, 0.0, v.tw.end);
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
  assert(current_row == nb_constraints + 1);

  // Set variables and coefficients.
  const unsigned start_X_col = 2 * n + 4 + 1;
  const unsigned start_delta_col = start_X_col + K;
  const unsigned nb_var = start_delta_col + n;
  glp_add_cols(lp, nb_var);

  unsigned current_col = 1;
  i = 0;
  // Variables for time of services (t_i values).
  if (!v.has_start()) {
    // Ghost step not included in steps.
    auto name = "t" + std::to_string(i);
    glp_set_col_name(lp, current_col, name.c_str());
    glp_set_col_bnds(lp, current_col, GLP_LO, 0.0, 0.0);
    ++i;
    ++current_col;
  }
  for (const auto& step : steps) {
    auto name = "t" + std::to_string(i);
    glp_set_col_name(lp, current_col, name.c_str());

    if (step.forced_service.at.has_value()) {
      // Fixed t_i value.
      double service_at = static_cast<double>(step.forced_service.at.value());
      glp_set_col_bnds(lp, current_col, GLP_FX, service_at, service_at);
    } else {
      // t_i value has a lower bound, either 0 or user-defined.
      double LB = (step.forced_service.after.has_value())
                    ? static_cast<double>(step.forced_service.after.value())
                    : 0.0;
      if (step.forced_service.before.has_value()) {
        // t_i value has a user-defined upper bound.
        double UB = static_cast<double>(step.forced_service.before.value());
        glp_set_col_bnds(lp, current_col, GLP_DB, LB, UB);
      } else {
        // No upper bound for t_i value.
        glp_set_col_bnds(lp, current_col, GLP_LO, LB, 0.0);
      }
    }
    ++i;
    ++current_col;
  }
  if (!v.has_end()) {
    // Ghost step not included in steps.
    auto name = "t" + std::to_string(i);
    glp_set_col_name(lp, current_col, name.c_str());
    glp_set_col_bnds(lp, current_col, GLP_LO, 0.0, 0.0);
    ++i;
    ++current_col;
  }
  assert(current_col == n + 3);

  // Set makespan and sum(t_i - t_0) in objective.
  glp_set_obj_coef(lp, 1, -M2 - static_cast<double>(n));
  glp_set_obj_coef(lp, n + 2, M2);
  for (unsigned i = 2; i <= n + 1; ++i) {
    glp_set_obj_coef(lp, i, 1.0);
  }

  // Define variables for measure of TW violation and set in
  // objective.
  for (unsigned i = 0; i <= n + 1; ++i) {
    auto name = "Y" + std::to_string(i);
    glp_set_col_name(lp, current_col, name.c_str());
    glp_set_col_bnds(lp, current_col, GLP_LO, 0.0, 0.0);
    glp_set_obj_coef(lp, current_col, M1);
    ++current_col;
  }
  assert(current_col == 2 * n + 5);

  // Binary variables for job time window choice.
  for (unsigned i = 0; i < n; ++i) {
    const auto& step = steps[i + first_task_rank];
    const auto& tws = (step.type == STEP_TYPE::JOB) ? input.jobs[step.rank].tws
                                                    : v.breaks[step.rank].tws;
    for (unsigned k = 0; k < tws.size(); ++k) {
      auto name = "X" + std::to_string(i + 1) + "_" + std::to_string(k);
      glp_set_col_name(lp, current_col, name.c_str());
      glp_set_col_kind(lp, current_col, GLP_BV);
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

  // a[constraint_rank, n + 3] = 1
  ia[r] = constraint_rank;
  ja[r] = n + 3;
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

    const auto& step = steps[i + first_task_rank];
    const auto& tws = (step.type == STEP_TYPE::JOB) ? input.jobs[step.rank].tws
                                                    : v.breaks[step.rank].tws;
    for (const auto& tw : tws) {
      // a[constraint_rank, current_X_rank] = - earliest_date for k-th
      // TW of task.
      ia[r] = constraint_rank;
      ja[r] = current_X_rank;
      ar[r] = -static_cast<double>(tw.start);
      ++r;

      ++current_X_rank;
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

    const auto& step = steps[i + first_task_rank];
    const auto& tws = (step.type == STEP_TYPE::JOB) ? input.jobs[step.rank].tws
                                                    : v.breaks[step.rank].tws;
    for (const auto& tw : tws) {
      // a[constraint_rank, current_X_rank] = - latest_date for k-th
      // TW of task.
      ia[r] = constraint_rank;
      ja[r] = current_X_rank;
      ar[r] = -static_cast<double>(tw.end);
      ++r;

      ++current_X_rank;
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
    const auto& step = steps[i + first_task_rank];
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

  // Delta_i constraints.
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
  assert(constraint_rank == nb_constraints + 1);

  assert(r == nb_non_zero + 1);

  glp_load_matrix(lp, nb_non_zero, ia, ja, ar);

  delete[] ia;
  delete[] ja;
  delete[] ar;

  // Solve.
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

  // glp_write_lp(lp, NULL, "mip.lp");
  // glp_print_mip(lp, "mip.sol");

  // Get output.
  auto v_start = glp_mip_col_val(lp, 1);
  auto v_end = glp_mip_col_val(lp, n + 2);
  auto start_lead_time = glp_mip_col_val(lp, n + 3);
  auto end_delay = glp_mip_col_val(lp, 2 * n + 4);
  auto start_travel = glp_mip_col_val(lp, start_delta_col);

  std::vector<Duration> task_ETA;
  std::vector<Duration> task_travels;
  for (unsigned i = 0; i < n; ++i) {
    task_ETA.push_back(glp_mip_col_val(lp, i + 2));
    task_travels.push_back(glp_mip_col_val(lp, start_delta_col + 1 + i));
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
        auto val = glp_mip_col_val(lp, current_X_rank);
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
        auto val = glp_mip_col_val(lp, current_X_rank);
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

  glp_delete_prob(lp);
  glp_free_env();

  // Generate route.
  Cost duration = 0;
  Duration service = 0;
  Duration forward_wt = 0;
  Priority priority = 0;
  Amount sum_pickups(input.zero_amount());
  Amount sum_deliveries(input.zero_amount());
  Duration lead_time = 0;
  Duration delay = 0;
  std::unordered_set<VIOLATION> v_types;

  // Startup load is the sum of deliveries for jobs.
  Amount current_load(input.zero_amount());
  for (const auto& step : steps) {
    if (step.type == STEP_TYPE::JOB and step.job_type == JOB_TYPE::SINGLE) {
      current_load += input.jobs[step.rank].delivery;
    }
  }

  bool previous_over_capacity = !(current_load <= v.capacity);

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

  if (v.has_start()) {
    sol_steps.emplace_back(STEP_TYPE::START, v.start.value(), current_load);
    sol_steps.back().duration = 0;
    sol_steps.back().arrival = v_start;
    if (v_start < v.tw.start) {
      sol_steps.back().violations.types.insert(VIOLATION::LEAD_TIME);
      v_types.insert(VIOLATION::LEAD_TIME);
      Duration lt = v.tw.start - v_start;
      sol_steps.back().violations.lead_time = lt;
      lead_time += lt;
    }

    if (previous_over_capacity) {
      sol_steps.back().violations.types.insert(VIOLATION::LOAD);
      v_types.insert(VIOLATION::LOAD);
    }
  } else {
    // Vehicle time window violation at startup is not reported in
    // steps as there is no start step.
    lead_time += start_lead_time;
  }

  Duration previous_start = v_start;
  Duration previous_service = 0;
  Duration previous_travel = start_travel;
  unsigned task_rank = 0;

  for (const auto& step : steps) {
    switch (step.type) {
    case STEP_TYPE::START:
      continue;
      break;
    case STEP_TYPE::JOB: {
      auto job_rank = step.rank;
      const auto& job = input.jobs[job_rank];

      service += job.service;
      priority += job.priority;

      current_load += job.pickup;
      current_load -= job.delivery;
      sum_pickups += job.pickup;
      sum_deliveries += job.delivery;

      sol_steps.emplace_back(job, current_load);
      auto& current = sol_steps.back();

      duration += previous_travel;
      current.duration = duration;

      auto arrival = previous_start + previous_service + previous_travel;
      auto service_start = task_ETA[task_rank];
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
      bool over_capacity = !(current_load <= v.capacity);
      if (previous_over_capacity or over_capacity) {
        current.violations.types.insert(VIOLATION::LOAD);
        v_types.insert(VIOLATION::LOAD);
      }
      previous_over_capacity = over_capacity;
      if (!input.vehicle_ok_with_job(vehicle_rank, job_rank)) {
        current.violations.types.insert(VIOLATION::SKILLS);
        v_types.insert(VIOLATION::SKILLS);
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

      unassigned_ranks.erase(job_rank);
      previous_start = service_start;
      previous_service = job.service;
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

      auto arrival = previous_start + previous_service + previous_travel;
      auto service_start = task_ETA[task_rank];
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
      if (previous_over_capacity) {
        current.violations.types.insert(VIOLATION::LOAD);
        v_types.insert(VIOLATION::LOAD);
      }

      previous_start = service_start;
      previous_service = b.service;
      previous_travel = task_travels[task_rank];
      ++task_rank;
      break;
    }
    case STEP_TYPE::END:
      duration += previous_travel;

      assert(previous_start + previous_service + previous_travel == v_end);

      sol_steps.emplace_back(STEP_TYPE::END, v.end.value(), current_load);
      sol_steps.back().duration = duration;
      sol_steps.back().arrival = v_end;

      if (v.tw.end < v_end) {
        sol_steps.back().violations.types.insert(VIOLATION::DELAY);
        v_types.insert(VIOLATION::DELAY);
        Duration dl = v_end - v.tw.end;
        sol_steps.back().violations.delay = dl;
        delay += dl;
      }
      if (previous_over_capacity) {
        sol_steps.back().violations.types.insert(VIOLATION::LOAD);
        v_types.insert(VIOLATION::LOAD);
      }
      break;
    }
  }

  if (!v.has_end()) {
    // Vehicle time window violation on route end is not reported in
    // steps as there is no end step.
    delay += end_delay;
  }

  assert(!v.has_start() or
         start_lead_time == sol_steps.front().violations.lead_time);
  assert(!v.has_end() or end_delay == sol_steps.back().violations.delay);

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
               duration,
               service,
               duration,
               forward_wt,
               priority,
               sum_deliveries,
               sum_pickups,
               v.description,
               std::move(Violations(lead_time,
                                    delay,
                                    start_lead_time,
                                    end_delay,
                                    std::move(v_types))));
}

} // namespace validation
} // namespace vroom
