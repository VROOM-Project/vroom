/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <iostream>

#include "algorithms/validation/choose_invalid.h"

#include <glpk.h>

namespace vroom {
namespace validation {

Route choose_invalid_route(const Input& input,
                           unsigned vehicle_rank,
                           const std::vector<Index>& job_ranks,
                           std::unordered_set<Index>& unassigned_ranks) {
  const auto& m = input.get_matrix();
  unsigned n = job_ranks.size();
  const auto& v = input.vehicles[vehicle_rank];

  constexpr double M = 10.0;

  // Create problem.
  glp_prob* lp;
  lp = glp_create_prob();
  glp_set_prob_name(lp, "choose_ETA");
  glp_set_obj_dir(lp, GLP_MIN);

  // Define constraints.
  unsigned nb_constraints = 3 * n + 1;
  double first = 0;
  if (v.has_start()) {
    ++nb_constraints;
    // Take into account time from start point to first job.
    first = m[v.start.value().index()][input.jobs[job_ranks.front()].index()];
  }
  double last = 0;
  if (v.has_end()) {
    ++nb_constraints;
    // Take into account time from last job arrival to end point.
    last = m[input.jobs[job_ranks.back()].index()][v.end.value().index()] +
           input.jobs[job_ranks.back()].service;
  }
  glp_add_rows(lp, nb_constraints);

  unsigned current_row = 1;
  // Precedence constraints.
  glp_set_row_name(lp, current_row, "p0");
  glp_set_row_bnds(lp, current_row, GLP_LO, first, 0.0);
  ++current_row;

  for (unsigned i = 0; i < n - 1; ++i) {
    double dist =
      m[input.jobs[job_ranks[i]].index()][input.jobs[job_ranks[i + 1]].index()];
    double service = input.jobs[job_ranks[i]].service;
    auto name = "p" + std::to_string(i + 1);
    glp_set_row_name(lp, current_row, name.c_str());
    glp_set_row_bnds(lp, current_row, GLP_LO, dist + service, 0.0);
    ++current_row;
  }
  glp_set_row_name(lp, current_row, ("p" + std::to_string(n)).c_str());
  glp_set_row_bnds(lp, current_row, GLP_LO, last, 0.0);
  ++current_row;

  // Vehicle TW start violation constraint.
  if (v.has_start()) {
    assert(current_row == n + 2);
    double lb = v.tw.start;
    glp_set_row_name(lp, current_row, "e0");
    glp_set_row_bnds(lp, current_row, GLP_LO, lb, 0.0);
    ++current_row;
  }

  // "Earliest violation" constraints.
  for (unsigned i = 0; i < n; ++i) {
    double lb = input.jobs[job_ranks[i]].tws[0].start;
    auto name = "e" + std::to_string(i + 1);
    glp_set_row_name(lp, current_row, name.c_str());
    glp_set_row_bnds(lp, current_row, GLP_LO, lb, 0.0);
    ++current_row;
  }

  // "Latest violation" constraints.
  for (unsigned i = 0; i < n; ++i) {
    double ub = input.jobs[job_ranks[i]].tws[0].end;
    auto name = "l" + std::to_string(i + 1);
    glp_set_row_name(lp, current_row, name.c_str());
    glp_set_row_bnds(lp, current_row, GLP_UP, 0.0, ub);
    ++current_row;
  }

  // Vehicle TW end violation constraint.
  if (v.has_end()) {
    double ub = v.tw.end;
    auto name = "l" + std::to_string(n + 1);
    glp_set_row_name(lp, current_row, name.c_str());
    glp_set_row_bnds(lp, current_row, GLP_UP, 0.0, ub);
    ++current_row;
  }

  assert(current_row == nb_constraints + 1);

  // Set variables and coefficients.
  unsigned nb_var = 2 * n + 4;
  glp_add_cols(lp, nb_var);

  unsigned current_col = 1;
  // Wanabee ETA.
  for (unsigned i = 0; i <= n + 1; ++i) {
    auto name = "t" + std::to_string(i);
    glp_set_col_name(lp, current_col, name.c_str());
    glp_set_col_bnds(lp, current_col, GLP_LO, 0.0, 0.0);
    ++current_col;
  }

  // Set makespan in objective.
  glp_set_obj_coef(lp, 1, -1.0);
  glp_set_obj_coef(lp, n + 2, 1.0);

  // Define variables for measure of TW violation and set in
  // objective.
  for (unsigned i = 0; i <= n + 1; ++i) {
    auto name = "Y" + std::to_string(i);
    glp_set_col_name(lp, current_col, name.c_str());
    glp_set_col_bnds(lp, current_col, GLP_LO, 0.0, 0.0);
    glp_set_obj_coef(lp, current_col, M);
    ++current_col;
  }

  assert(current_col == nb_var + 1);

  // Define non-zero elements in matrix.
  unsigned nb_non_zero = 2 * nb_constraints;
  int* ia = new int[1 + nb_non_zero];
  int* ja = new int[1 + nb_non_zero];
  double* ar = new double[1 + nb_non_zero];

  unsigned current_rank = 1;
  // Coefficients for precedence constraints.
  for (unsigned i = 1; i <= n + 1; ++i) {
    // a[i,i] = -1
    ia[current_rank] = i;
    ja[current_rank] = i;
    ar[current_rank] = -1;
    ++current_rank;

    // a[i,i + 1] = 1
    ia[current_rank] = i;
    ja[current_rank] = i + 1;
    ar[current_rank] = 1;
    ++current_rank;
  }

  unsigned constraint_rank = n + 2;

  // Coefficients for vehicle "earliest violation" constraint.
  if (v.has_start()) {
    // a[constraint_rank, 1] = 1
    ia[current_rank] = constraint_rank;
    ja[current_rank] = 1;
    ar[current_rank] = 1;
    ++current_rank;

    // a[constraint_rank, n + 3] = 1
    ia[current_rank] = constraint_rank;
    ja[current_rank] = n + 3;
    ar[current_rank] = 1;
    ++current_rank;

    ++constraint_rank;
  }

  // Coefficients for "earliest violations" constraints.
  for (unsigned i = 2; i <= n + 1; ++i) {
    // a[constraint_rank, i] = 1
    ia[current_rank] = constraint_rank;
    ja[current_rank] = i;
    ar[current_rank] = 1;
    ++current_rank;

    // a[constraint_rank, n + 2 + i] = 1
    ia[current_rank] = constraint_rank;
    ja[current_rank] = n + 2 + i;
    ar[current_rank] = 1;
    ++current_rank;

    ++constraint_rank;
  }

  // Coefficients for "latest violations" constraints.
  for (unsigned i = 2; i <= n + 1; ++i) {
    // a[constraint_rank, i] = 1
    ia[current_rank] = constraint_rank;
    ja[current_rank] = i;
    ar[current_rank] = 1;
    ++current_rank;

    // a[n + i, n + 2 + i] = -1
    ia[current_rank] = constraint_rank;
    ja[current_rank] = n + 2 + i;
    ar[current_rank] = -1;
    ++current_rank;

    ++constraint_rank;
  }

  // Coefficients for vehicle "latest violation" constraint.
  if (v.has_end()) {
    // a[constraint_rank, n + 2] = 1
    ia[current_rank] = constraint_rank;
    ja[current_rank] = n + 2;
    ar[current_rank] = 1;
    ++current_rank;

    // a[constraint_rank, 2 * n + 4] = -1
    ia[current_rank] = constraint_rank;
    ja[current_rank] = 2 * n + 4;
    ar[current_rank] = -1;
    ++current_rank;

    ++constraint_rank;
  }

  assert(constraint_rank == nb_constraints + 1);
  assert(current_rank == nb_non_zero + 1);

  glp_load_matrix(lp, nb_non_zero, ia, ja, ar);

  delete[] ia;
  delete[] ja;
  delete[] ar;

  // Solve.
  glp_simplex(lp, NULL);

  // Get output.
  std::cout << "- Obj: " << glp_get_obj_val(lp) << std::endl;

  auto v_start = glp_get_col_prim(lp, 1);
  auto v_end = glp_get_col_prim(lp, n + 2);

  std::cout << "start: " << v_start << std::endl;
  std::vector<Duration> job_ETA;
  for (unsigned i = 2; i <= n + 1; ++i) {
    job_ETA.push_back(glp_get_col_prim(lp, i));
    std::cout << "t" << i - 1 << ": " << job_ETA.back() << std::endl;
  }
  std::cout << "end: " << v_end << std::endl;

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

  // Startup load is the sum of deliveries for jobs.
  Amount current_load(input.zero_amount());
  for (const auto job_rank : job_ranks) {
    const auto& job = input.jobs[job_rank];
    if (job.type == JOB_TYPE::SINGLE) {
      current_load += job.delivery;
    }
  }

  std::vector<Step> steps;

  Duration next_arrival;

  if (v.has_start()) {
    steps.emplace_back(STEP_TYPE::START, v.start.value(), current_load);
    steps.back().duration = 0;
    steps.back().arrival = v_start;
    if (v_start < v.tw.start) {
      Duration lt = v.tw.start - v_start;
      steps.back().lead_time = lt;
      lead_time += lt;
    }

    auto travel =
      m[v.start.value().index()][input.jobs[job_ranks.front()].index()];
    duration += travel;
    next_arrival = v_start + travel;
  } else {
    next_arrival = job_ETA.front();
  }

  auto& first_job = input.jobs[job_ranks.front()];
  service += first_job.service;
  priority += first_job.priority;

  current_load += first_job.pickup;
  current_load -= first_job.delivery;
  sum_pickups += first_job.pickup;
  sum_deliveries += first_job.delivery;

  steps.emplace_back(first_job, current_load);
  auto& first_step = steps.back();
  first_step.duration = duration;
  auto service_start = job_ETA.front();
  assert(next_arrival <= service_start);

  first_step.arrival = next_arrival;

  Duration wt = service_start - next_arrival;
  first_step.waiting_time = wt;
  forward_wt += wt;

  if (service_start < first_job.tws[0].start) {
    Duration lt = first_job.tws[0].start - service_start;
    first_step.lead_time = lt;
    lead_time += lt;
  }
  if (first_job.tws[0].end < service_start) {
    Duration dl = service_start - first_job.tws[0].end;
    first_step.delay = dl;
    delay += dl;
  }

  unassigned_ranks.erase(job_ranks.front());

  for (std::size_t r = 0; r < job_ranks.size() - 1; ++r) {
    const auto& previous_job = input.jobs[job_ranks[r]];
    const auto& next_job = input.jobs[job_ranks[r + 1]];

    Duration travel = m[previous_job.index()][next_job.index()];
    duration += travel;
    next_arrival = service_start + previous_job.service + travel;

    service += next_job.service;
    priority += next_job.priority;

    current_load += next_job.pickup;
    current_load -= next_job.delivery;
    sum_pickups += next_job.pickup;
    sum_deliveries += next_job.delivery;

    steps.emplace_back(input.jobs[job_ranks[r + 1]], current_load);
    auto& current = steps.back();
    current.duration = duration;

    service_start = job_ETA[r + 1];
    assert(next_arrival <= service_start);

    current.arrival = next_arrival;
    Duration wt = service_start - next_arrival;
    current.waiting_time = wt;
    forward_wt += wt;

    if (service_start < next_job.tws[0].start) {
      Duration lt = next_job.tws[0].start - service_start;
      current.lead_time = lt;
      lead_time += lt;
    }
    if (next_job.tws[0].end < service_start) {
      Duration dl = service_start - next_job.tws[0].end;
      current.delay = dl;
      delay += dl;
    }

    unassigned_ranks.erase(job_ranks[r + 1]);
  }

  if (v.has_end()) {
    const auto& last_job = input.jobs[job_ranks.back()];
    Duration travel = m[last_job.index()][v.end.value().index()];
    duration += travel;

    assert(service_start + last_job.service + travel == v_end);

    steps.emplace_back(STEP_TYPE::END, v.end.value(), current_load);
    steps.back().duration = duration;
    steps.back().arrival = v_end;

    if (v.tw.end < v_end) {
      Duration dl = v_end - v.tw.end;
      steps.back().delay = dl;
      delay += dl;
    }
  }

  return Route(v.id,
               std::move(steps),
               duration,
               service,
               duration,
               forward_wt,
               priority,
               sum_deliveries,
               sum_pickups,
               v.description,
               lead_time,
               delay);
}

} // namespace validation
} // namespace vroom
