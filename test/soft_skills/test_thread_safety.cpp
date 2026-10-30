// Soft-skills thread-safety tests (GAP-10 companion).
//
// Solves the deterministic fixtures with multi-threaded local search
// and checks that preference counts and placements are consistent
// across repeats and across concurrent Input instances:
//
//   * repeated solve(exploration level, 2 threads) runs of fixtures
//     01/03/04 must produce identical fingerprints (serving vehicle,
//     step sequence, preference counters, summary cost);
//   * a concurrent burst of threads, each running its own two-thread
//     solve of fixture 01, must all agree with the single-thread
//     reference (authoritative rescore after the cross-thread merge);
//   * fixtures where several vehicles are equally good (02) may vary
//     in placement, so only the preference totals are pinned there.
//
// Returns true iff every check passes.

#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution/solution.h"
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

std::string read_fixture(const std::string& name) {
  std::ifstream in(fixture_path(name));
  if (!in.is_open()) {
    return "";
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

// Deterministic summary of a solution: routed vehicle/step structure,
// preference counters and total cost/duration/distance. Deliberately
// includes the full step sequence: the fixtures are small enough that
// any placement freedom is exercised by fixture 02, where the
// fingerprint comparison is restricted to the preference totals.
std::string fingerprint(const vroom::Solution& sol) {
  std::ostringstream out;
  out << "cost=" << sol.summary.cost << ";duration=" << sol.summary.duration
      << ";distance=" << sol.summary.distance
      << ";unassigned=" << sol.unassigned.size() << ";";
  for (const auto& route : sol.routes) {
    out << "vehicle=" << route.vehicle << ",steps=[";
    for (const auto& step : route.steps) {
      out << static_cast<unsigned>(step.step_type) << ":" << step.id << " ";
    }
    out << "]";
    if (route.soft_skill_preference.has_value()) {
      const auto& pref = route.soft_skill_preference.value();
      out << ",pref=" << pref.eligible << "/" << pref.satisfied << "/"
          << pref.unsatisfied;
    }
    out << ";";
  }
  if (sol.summary.soft_skill_preference.has_value()) {
    const auto& pref = sol.summary.soft_skill_preference.value();
    out << "summary_pref=" << pref.eligible << "/" << pref.satisfied << "/"
        << pref.unsatisfied;
  } else {
    out << "summary_pref=none";
  }
  return out.str();
}

// Solve one fixture with nb_thread worker threads. Returns the
// solution, or nullopt with a printed failure.
std::optional<vroom::Solution> solve_threaded(const std::string& file,
                                              unsigned nb_thread,
                                              const std::string& label) {
  const auto json = read_fixture(file);
  if (json.empty()) {
    std::cerr << "[thread-safety] FAIL " << label << ": fixture not found"
              << std::endl;
    return std::nullopt;
  }
  try {
    vroom::Input input;
    vroom::io::parse(input, json, false);
    return input.solve(vroom::DEFAULT_EXPLORATION_LEVEL, nb_thread);
  } catch (const vroom::Exception& e) {
    std::cerr << "[thread-safety] FAIL " << label << ": " << e.message
              << std::endl;
    return std::nullopt;
  }
}

bool expect_true(const std::string& label,
                 const std::string& what,
                 bool condition) {
  if (!condition) {
    std::cerr << "[thread-safety] FAIL " << label << ": " << what
              << std::endl;
    return false;
  }
  return true;
}

// Repeated multi-threaded runs must share one fingerprint. When
// pin_summary_pref is set, that exact "e/s/u" summary preference is
// also required on every run.
bool check_repeats(const std::string& file,
                   const std::string& label,
                   unsigned nb_thread,
                   unsigned repeats,
                   const std::string& expected_summary_pref) {
  bool ok = true;
  std::string reference;
  for (unsigned run = 0; run < repeats; ++run) {
    const auto sol = solve_threaded(file, nb_thread, label);
    ok = ok && expect_true(label, "run " + std::to_string(run) + " failed",
                           sol.has_value());
    if (!sol.has_value()) {
      return false;
    }
    const auto fp = fingerprint(*sol);
    if (run == 0) {
      reference = fp;
    } else {
      ok = ok && expect_true(label,
                             "run " + std::to_string(run) +
                               " fingerprint differs from the first run",
                             fp == reference);
    }
    ok = ok &&
         expect_true(label, "summary preference mismatch on run " +
                             std::to_string(run),
                     fp.find("summary_pref=" + expected_summary_pref) !=
                       std::string::npos);
  }
  return ok;
}

// Concurrent solves of the same fixture from independent threads,
// each with its own Input and two worker threads.
bool check_concurrent_burst(const std::string& file,
                            const std::string& label,
                            unsigned nb_threads,
                            const std::string& expected_fingerprint_fragment) {
  std::vector<std::string> results(nb_threads);
  std::vector<std::thread> pool;
  pool.reserve(nb_threads);
  for (unsigned t = 0; t < nb_threads; ++t) {
    pool.emplace_back([&file, &results, t]() {
      try {
        vroom::Input input;
        vroom::io::parse(input, read_fixture(file), false);
        const auto sol = input.solve(vroom::DEFAULT_EXPLORATION_LEVEL, 2);
        results[t] = fingerprint(sol);
      } catch (const vroom::Exception& e) {
        results[t] = "exception: " + e.message;
      }
    });
  }
  for (auto& worker : pool) {
    worker.join();
  }

  bool ok = true;
  for (unsigned t = 0; t < nb_threads; ++t) {
    ok = ok && expect_true(label,
                           "concurrent thread " + std::to_string(t) +
                             " result differs from the others",
                           results[t] == results[0]);
    ok = ok && expect_true(
             label,
             "concurrent thread " + std::to_string(t) +
               " output lacks the expected structure",
             results[t].find(expected_fingerprint_fragment) !=
               std::string::npos);
  }
  return ok;
}

} // namespace

bool run_thread_safety_tests() {
  std::size_t total = 0;
  std::size_t failures = 0;
  const auto check = [&](bool passed) {
    ++total;
    if (!passed) {
      ++failures;
    }
  };

  // Baseline agreement: single-thread reference for fixture 01 must
  // match the multi-thread fingerprint, then repeat under
  // concurrency.
  {
    const std::string label = "01 two-thread repeats";
    const auto ref = solve_threaded(
      "vroom_soft_skill_fixtures/01_all_vs_any.json", 1, label);
    bool ok = expect_true(label, "reference solve failed", ref.has_value());
    if (ok) {
      const auto ref_fp = fingerprint(*ref);
      ok = expect_true(label,
                       "single-thread reference does not match the "
                       "documented vehicle-2 / 1-1-0 outcome",
                       ref_fp.find("vehicle=2") != std::string::npos &&
                         ref_fp.find("summary_pref=1/1/0") !=
                           std::string::npos);
    }
    ok = ok && check_repeats("vroom_soft_skill_fixtures/01_all_vs_any.json",
                             label, 2, 4, "1/1/0");
    if (ok) {
      // Two-thread result must equal the single-thread reference for
      // this fixture (both vehicles are cost-equivalent, preference
      // decides, the merge rescores authoritatively).
      const auto two = solve_threaded(
        "vroom_soft_skill_fixtures/01_all_vs_any.json", 2, label);
      ok = two.has_value() &&
           expect_true(label,
                       "two-thread fingerprint differs from single thread",
                       fingerprint(*two) == fingerprint(*ref));
    }
    check(ok);
  }

  // Shipment fixture: pickup/delivery stay paired on vehicle 1 under
  // multi-threaded solving.
  {
    const std::string label = "03 two-thread shipment repeats";
    bool ok = check_repeats(
      "vroom_soft_skill_fixtures/03_shipment_override.json", label, 2, 3,
      "1/1/0");
    if (ok) {
      const auto sol =
        solve_threaded("vroom_soft_skill_fixtures/03_shipment_override.json", 2,
                       label);
      ok = sol.has_value() && sol->routes.size() == 1 &&
           expect_true(label, "shipment left vehicle 1 under concurrency",
                       sol->routes.front().vehicle == 1);
    }
    check(ok);
  }

  // Hard-eligibility fixture: only vehicle 2 may serve, also when the
  // search runs multi-threaded.
  check(check_repeats("vroom_soft_skill_fixtures/"
                      "04_hard_skills_override_soft.json",
                      "04 two-thread hard eligibility", 2, 3, "1/1/0"));

  // Fixture 02 admits both vehicles: only the preference totals are
  // pinned across runs (placement freedom, no brittle order checks).
  {
    const std::string label = "02 preference totals stable";
    bool ok = true;
    for (unsigned run = 0; run < 3; ++run) {
      const auto sol = solve_threaded(
        "vroom_soft_skill_fixtures/02_any_override.json", 2, label);
      ok = ok && sol.has_value() &&
           sol->summary.soft_skill_preference.has_value() &&
           sol->summary.soft_skill_preference->eligible == 1 &&
           sol->summary.soft_skill_preference->satisfied == 1 &&
           sol->summary.soft_skill_preference->unsatisfied == 0 &&
           sol->unassigned.empty();
      if (!ok) {
        std::cerr << "[thread-safety] FAIL " << label << ": totals drifted on "
                  << "run " << run << std::endl;
        break;
      }
    }
    check(ok);
  }

  // Concurrent independent solves (each with 2 worker threads) must
  // all agree on the exact fingerprint.
  check(check_concurrent_burst("vroom_soft_skill_fixtures/"
                               "01_all_vs_any.json",
                               "01 concurrent burst", 4, "vehicle=2"));

  std::cout << (total - failures) << "/" << total
            << " thread-safety tests passed" << std::endl;
  return failures == 0 && total > 0;
}
