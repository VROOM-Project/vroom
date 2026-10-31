// Soft-skills solver integration tests on the deterministic fixtures
// 00-06. Every assertion targets semantics and hard-constraint
// invariants (eligibility, capacity, precedence, preference counts),
// never brittle exact tour order.
//
//   00: upstream baseline solves cleanly, no preference reporting
//   01: under "all" only vehicle 2 fully matches [101,202]
//   02: under "any", either vehicle is fine, preference satisfied
//   03: shipment rides the fully matching vehicle 1, pickup before
//       delivery
//   04: hard skills decide eligibility, only vehicle 2 can serve; the
//       soft match of vehicle 1 is irrelevant
//   05: capacity rules the job out, preference cannot override it
//   06: empty task skills stay neutral
//
// Returns true iff every fixture check passes.

#include <algorithm>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/job.h"
#include "structures/vroom/solution/solution.h"
#include "structures/vroom/vehicle.h"
#include "utils/exception.h"
#include "utils/input_parser.h"

namespace {

std::string fixture_path(const std::string& name) {
  std::ifstream probe(name);
  if (probe.good()) {
    return name;
  }
  return "../../" + name;
}

// Parse and solve a fixture file. Returns the solution on success
// (after printing details on any load or solve failure).
std::optional<vroom::Solution> solve_fixture(const std::string& name,
                                            vroom::Input& input,
                                            const std::string& label,
                                            unsigned nb_thread = 1) {
  std::ifstream in(fixture_path(name));
  if (!in.is_open()) {
    std::cerr << "[solver] FAIL " << label << ": cannot open " << name
              << std::endl;
    return std::nullopt;
  }
  std::stringstream buffer;
  buffer << in.rdbuf();

  try {
    vroom::io::parse(input, buffer.str(), false);
    return input.solve(vroom::DEFAULT_EXPLORATION_LEVEL, nb_thread);
  } catch (const vroom::Exception& e) {
    std::cerr << "[solver] FAIL " << label << ": " << e.message << std::endl;
    return std::nullopt;
  }
}

const vroom::Route* route_for_vehicle(const vroom::Solution& sol,
                                      vroom::Id vehicle) {
  for (const auto& route : sol.routes) {
    if (route.vehicle == vehicle) {
      return &route;
    }
  }
  return nullptr;
}

// Vehicle serving the task with the given id (jobs and pickup
// endpoints), or 0 when the task is not routed.
vroom::Id vehicle_of(const vroom::Solution& sol, vroom::Id task_id) {
  for (const auto& route : sol.routes) {
    for (const auto& step : route.steps) {
      if (step.step_type == vroom::STEP_TYPE::JOB && step.id == task_id &&
          step.job_type != vroom::JOB_TYPE::DELIVERY) {
        return route.vehicle;
      }
    }
  }
  return 0;
}

bool has_step(const vroom::Route& route, vroom::Id step_id) {
  return std::any_of(route.steps.begin(),
                     route.steps.end(),
                     [&](const auto& s) { return s.id == step_id; });
}

// Position of a task step inside the route (step index), or the number
// of steps when absent.
std::size_t step_position(const vroom::Route& route, vroom::Id step_id) {
  for (std::size_t i = 0; i < route.steps.size(); ++i) {
    if (route.steps[i].id == step_id &&
        route.steps[i].step_type == vroom::STEP_TYPE::JOB) {
      return i;
    }
  }
  return route.steps.size();
}

const vroom::Job* find_job(const vroom::Input& input,
                           vroom::Id id,
                           bool delivery = false) {
  for (const auto& job : input.jobs) {
    const bool wanted = delivery ? (job.type == vroom::JOB_TYPE::DELIVERY)
                                 : (job.type != vroom::JOB_TYPE::DELIVERY);
    if (job.id == id && wanted) {
      return &job;
    }
  }
  return nullptr;
}

const vroom::Vehicle* find_vehicle(const vroom::Input& input, vroom::Id id) {
  for (const auto& vehicle : input.vehicles) {
    if (vehicle.id == id) {
      return &vehicle;
    }
  }
  return nullptr;
}

bool expect_true(const std::string& label,
                 const std::string& what,
                 bool condition) {
  if (!condition) {
    std::cerr << "[solver] FAIL " << label << ": " << what << std::endl;
    return false;
  }
  return true;
}

bool expect_pref(const std::string& label,
                 const std::string& where,
                 const vroom::SoftSkillPreference& pref,
                 std::size_t eligible,
                 std::size_t satisfied,
                 std::size_t unsatisfied) {
  if (pref.eligible != eligible || pref.satisfied != satisfied ||
      pref.unsatisfied != unsatisfied) {
    std::cerr << "[solver] FAIL " << label << ": " << where
              << " preference is " << pref.eligible << "/" << pref.satisfied
              << "/" << pref.unsatisfied << ", expected " << eligible << "/"
              << satisfied << "/" << unsatisfied << std::endl;
    return false;
  }
  return true;
}

// Hard-constraint invariants that must hold for every fixture: no hard
// skill violations, capacity respected, pickup before delivery on the
// same route, summary cost/duration/distance equal the route totals.
bool check_invariants(const std::string& label,
                      const vroom::Input& input,
                      const vroom::Solution& sol) {
  bool ok = true;

  vroom::Cost sum_cost = 0;
  vroom::Duration sum_duration = 0;
  vroom::Distance sum_distance = 0;

  for (const auto& route : sol.routes) {
    const auto* vehicle = find_vehicle(input, route.vehicle);
    ok = expect_true(label, "routed vehicle exists in input",
                     vehicle != nullptr);
    if (vehicle == nullptr) {
      continue;
    }

    bool seen_pickup = false;
    std::size_t pickup_position = 0;

    for (std::size_t s = 0; s < route.steps.size(); ++s) {
      const auto& step = route.steps[s];
      if (step.step_type != vroom::STEP_TYPE::JOB) {
        continue;
      }
      const bool is_delivery = step.job_type == vroom::JOB_TYPE::DELIVERY;
      const auto* job = find_job(input, step.id, is_delivery);
      ok = expect_true(label, "routed step refers to an input task",
                       job != nullptr);
      if (job == nullptr) {
        continue;
      }

      // Hard skills: every required skill must be held by the vehicle.
      const bool skills_ok =
        std::all_of(job->skills.begin(),
                    job->skills.end(),
                    [&](const auto& skill) {
                      return vehicle->skills.count(skill) != 0;
                    });
      ok = expect_true(label,
                       "vehicle " + std::to_string(vehicle->id) +
                         " breaks a hard skill requirement",
                       skills_ok);

      if (job->type == vroom::JOB_TYPE::PICKUP) {
        seen_pickup = true;
        pickup_position = s;
      } else if (job->type == vroom::JOB_TYPE::DELIVERY) {
        ok = expect_true(label,
                         "delivery routed without its pickup before it on "
                         "the same vehicle",
                         seen_pickup && pickup_position < s);
      }
    }

    // Capacity: aggregated route load per metric must fit.
    for (std::size_t m = 0; m < route.delivery.size(); ++m) {
      const bool fits = m < vehicle->capacity.size() &&
                        route.delivery[m] <= vehicle->capacity[m] &&
                        route.pickup[m] <= vehicle->capacity[m];
      ok = expect_true(label,
                       "route load exceeds vehicle capacity on metric " +
                         std::to_string(m),
                       fits);
    }

    sum_cost += route.cost;
    sum_duration += route.duration;
    sum_distance += route.distance;
  }

  ok = expect_true(label,
                   "summary cost does not match the sum of route costs",
                   sol.summary.cost == sum_cost);
  ok = expect_true(
    label, "summary duration does not match the sum of route durations",
    sol.summary.duration == sum_duration);
  ok = expect_true(
    label, "summary distance does not match the sum of route distances",
    sol.summary.distance == sum_distance);

  // Preference-reporting invariants: presence must agree between
  // summary and routes, per route eligible splits exactly, and the
  // summary holds the exact route sums.
  std::size_t sum_eligible = 0;
  std::size_t sum_satisfied = 0;
  std::size_t sum_unsatisfied = 0;
  for (const auto& route : sol.routes) {
    if (route.soft_skill_preference.has_value()) {
      const auto& pref = route.soft_skill_preference.value();
      ok = expect_true(label,
                       "route eligible != satisfied + unsatisfied",
                       pref.eligible == pref.satisfied + pref.unsatisfied);
      sum_eligible += pref.eligible;
      sum_satisfied += pref.satisfied;
      sum_unsatisfied += pref.unsatisfied;
    }
  }
  const bool summary_pref = sol.summary.soft_skill_preference.has_value();
  for (const auto& route : sol.routes) {
    ok = expect_true(
      label, "summary/route preference presence mismatch",
      route.soft_skill_preference.has_value() == summary_pref);
  }
  if (summary_pref) {
    const auto& pref = sol.summary.soft_skill_preference.value();
    ok = expect_true(label,
                     "summary totals do not match the sums of route totals",
                     pref.eligible == sum_eligible &&
                       pref.satisfied == sum_satisfied &&
                       pref.unsatisfied == sum_unsatisfied);
  }

  return ok;
}

} // namespace

bool run_solver_tests() {
  std::size_t total = 0;
  std::size_t failures = 0;
  const auto check = [&](bool passed) {
    ++total;
    if (!passed) {
      ++failures;
    }
  };

  // 00: upstream baseline solves cleanly, no preference reporting.
  {
    const std::string label = "00 baseline";
    vroom::Input input;
    const auto sol = solve_fixture(
      "vroom_soft_skill_fixtures/00_baseline_upstream.json", input, label);
    bool ok = sol.has_value();
    if (ok) {
      ok = expect_true(label, "job 101 left unassigned",
                       sol->unassigned.empty()) &&
           expect_true(label, "expected exactly one route",
                       sol->routes.size() == 1) &&
           expect_true(label,
                       "summary reports a preference without soft-skills "
                       "input",
                       !sol->summary.soft_skill_preference.has_value());
      for (const auto& route : sol->routes) {
        ok = ok && expect_true(label,
                               "route reports a preference without "
                               "soft-skills input",
                               !route.soft_skill_preference.has_value());
      }
      ok = ok && check_invariants(label, input, *sol);
    }
    check(ok);
  }

  // 01: under "all" only vehicle 2 fully matches [101,202].
  {
    const std::string label = "01 all vs any";
    vroom::Input input;
    const auto sol = solve_fixture(
      "vroom_soft_skill_fixtures/01_all_vs_any.json", input, label);
    bool ok = sol.has_value();
    if (ok) {
      ok = expect_true(label, "job 101 unassigned", sol->unassigned.empty()) &&
           expect_true(label, "job 101 not on vehicle 2",
                       vehicle_of(*sol, 101) == 2);
      const auto* route = route_for_vehicle(*sol, 2);
      ok = ok && expect_true(label, "vehicle 2 route missing",
                             route != nullptr);
      if (route != nullptr && route->soft_skill_preference.has_value()) {
        ok = ok && expect_pref(label, "vehicle 2 route",
                               *route->soft_skill_preference, 1, 1, 0);
      }
      ok = ok && sol->summary.soft_skill_preference.has_value() &&
           expect_pref(label, "summary", *sol->summary.soft_skill_preference, 1,
                       1, 0);
      ok = ok && check_invariants(label, input, *sol);
    }
    check(ok);
  }

  // 02: under "any" both vehicles match; any placement is valid and
  // the preference must be satisfied on the serving route.
  {
    const std::string label = "02 any override";
    vroom::Input input;
    const auto sol = solve_fixture(
      "vroom_soft_skill_fixtures/02_any_override.json", input, label);
    bool ok = sol.has_value();
    if (ok) {
      const auto serving = vehicle_of(*sol, 101);
      ok = expect_true(label, "job 101 unassigned", serving != 0) &&
           expect_true(label, "job 101 on unknown vehicle",
                       serving == 1 || serving == 2);
      const auto* route =
        serving != 0 ? route_for_vehicle(*sol, serving) : nullptr;
      ok = ok && expect_true(label, "serving route missing", route != nullptr);
      if (route != nullptr && route->soft_skill_preference.has_value()) {
        ok = ok && expect_pref(label, "serving route",
                               *route->soft_skill_preference, 1, 1, 0);
      }
      ok = ok && sol->summary.soft_skill_preference.has_value() &&
           expect_pref(label, "summary", *sol->summary.soft_skill_preference, 1,
                       1, 0);
      ok = ok && check_invariants(label, input, *sol);
    }
    check(ok);
  }

  // 03: shipment override "all" -> only vehicle 1 matches; pickup and
  // delivery stay on that vehicle, pickup first.
  {
    const std::string label = "03 shipment override";
    vroom::Input input;
    const auto sol = solve_fixture(
      "vroom_soft_skill_fixtures/03_shipment_override.json", input, label);
    bool ok = sol.has_value();
    if (ok) {
      ok = expect_true(label, "shipment unassigned",
                       sol->unassigned.empty()) &&
           expect_true(label, "shipment not on vehicle 1",
                       vehicle_of(*sol, 201) == 1);
      const auto* route = route_for_vehicle(*sol, 1);
      ok = ok && expect_true(label, "vehicle 1 route missing",
                             route != nullptr);
      if (route != nullptr) {
        ok = ok &&
             expect_true(label, "delivery not on the pickup vehicle",
                         has_step(*route, 202)) &&
             expect_true(label, "delivery precedes pickup",
                         step_position(*route, 201) <
                           step_position(*route, 202));
        if (route->soft_skill_preference.has_value()) {
          // The shipment counts once through its pickup endpoint.
          ok = ok && expect_pref(label, "vehicle 1 route",
                                 *route->soft_skill_preference, 1, 1, 0);
        }
      }
      ok = ok && sol->summary.soft_skill_preference.has_value() &&
           expect_pref(label, "summary", *sol->summary.soft_skill_preference, 1,
                       1, 0);
      ok = ok && check_invariants(label, input, *sol);
    }
    check(ok);
  }

  // 04: hard skill 8 makes vehicle 1 ineligible despite its perfect
  // soft match; the job must land on vehicle 2.
  {
    const std::string label = "04 hard skills override soft";
    vroom::Input input;
    const auto sol = solve_fixture(
      "vroom_soft_skill_fixtures/04_hard_skills_override_soft.json", input,
      label);
    bool ok = sol.has_value();
    if (ok) {
      ok = expect_true(label, "job 301 unassigned", sol->unassigned.empty()) &&
           expect_true(label, "job 301 not on the only hard-eligible "
                              "vehicle 2",
                       vehicle_of(*sol, 301) == 2) &&
           expect_true(label, "hard-ineligible vehicle 1 reports a route",
                       route_for_vehicle(*sol, 1) == nullptr);
      const auto* route = route_for_vehicle(*sol, 2);
      if (route != nullptr && route->soft_skill_preference.has_value()) {
        ok = ok && expect_pref(label, "vehicle 2 route",
                               *route->soft_skill_preference, 1, 1, 0);
      }
      ok = ok && check_invariants(label, input, *sol);
    }
    check(ok);
  }

  // 05: no vehicle has capacity; preference cannot make the job
  // feasible. Unassigned tasks never count as eligible.
  {
    const std::string label = "05 unassigned capacity";
    vroom::Input input;
    const auto sol = solve_fixture(
      "vroom_soft_skill_fixtures/05_unassigned_capacity.json", input, label);
    bool ok = sol.has_value();
    if (ok) {
      ok = expect_true(label, "job 101 was assigned although infeasible",
                       sol->unassigned.size() == 1) &&
           expect_true(label, "unexpected unassigned id",
                       !sol->unassigned.empty() &&
                         sol->unassigned[0].id == 101) &&
           expect_true(label, "routes reported for an all-unfeasible problem",
                       sol->routes.empty());
      ok = ok && sol->summary.soft_skill_preference.has_value() &&
           expect_pref(label, "summary", *sol->summary.soft_skill_preference, 0,
                       0, 0);
      ok = ok && check_invariants(label, input, *sol);
    }
    check(ok);
  }

  // 06: empty task skills are neutral: no preference applies, so all
  // counters stay at zero regardless of the vehicle skills present.
  {
    const std::string label = "06 empty skills neutral";
    vroom::Input input;
    const auto sol = solve_fixture(
      "vroom_soft_skill_fixtures/06_empty_skills_neutral.json", input, label);
    bool ok = sol.has_value();
    if (ok) {
      ok = expect_true(label, "job 101 unassigned", sol->unassigned.empty());
      ok = ok && sol->summary.soft_skill_preference.has_value() &&
           expect_pref(label, "summary", *sol->summary.soft_skill_preference, 0,
                       0, 0);
      ok = ok && check_invariants(label, input, *sol);
    }
    check(ok);
  }

  std::cout << (total - failures) << "/" << total
            << " solver tests passed" << std::endl;
  return failures == 0 && total > 0;
}
