// Soft-skills output JSON assertion tests (IS-10).
//
// Solves each fixture, renders the solution through
// vroom::io::to_json and checks the emitted document:
//
//   * solution code is 0 and the standard summary/route/unassigned
//     structure is intact;
//   * cost / duration / service / waiting-time fields in JSON match
//     the in-memory solution and route totals, with distances reported
//     whenever a distances matrix was provided, so the pre-existing
//     economics fields are unchanged by the soft-skills feature;
//   * "soft_skill_preference" is absent for the upstream baseline and
//     present (summary and every route) whenever soft-skills are in
//     play, with per-route eligible == satisfied + unsatisfied and
//     summary totals equal to the sum over routes;
//   * the exception serializer still emits {code, error}.
//
// Returns true iff every check passes.

#include <cstdint>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/solution/solution.h"
#include "utils/exception.h"
#include "utils/input_parser.h"
#include "utils/output_json.h"

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

bool expect_true(const std::string& label,
                 const std::string& what,
                 bool condition) {
  if (!condition) {
    std::cerr << "[output] FAIL " << label << ": " << what << std::endl;
    return false;
  }
  return true;
}

template <class T>
bool expect_int(const std::string& label,
                const std::string& what,
                const rapidjson::Value& holder,
                const char* key,
                T expected) {
  if (!holder.IsObject() || !holder.HasMember(key) ||
      !holder[key].IsInt64()) {
    std::cerr << "[output] FAIL " << label << ": missing or non-integer \""
              << key << "\" in " << what << std::endl;
    return false;
  }
  if (holder[key].GetInt64() != static_cast<std::int64_t>(expected)) {
    std::cerr << "[output] FAIL " << label << ": " << what << "." << key << " is "
              << holder[key].GetInt64() << ", expected " << expected
              << std::endl;
    return false;
  }
  return true;
}

bool expect_unsigned(const std::string& label,
                     const std::string& what,
                     const rapidjson::Value& holder,
                     const char* key,
                     std::uint64_t expected) {
  if (!holder.IsObject() || !holder.HasMember(key) ||
      !holder[key].IsUint64()) {
    std::cerr << "[output] FAIL " << label << ": missing or non-integer \""
              << key << "\" in " << what << std::endl;
    return false;
  }
  if (holder[key].GetUint64() != expected) {
    std::cerr << "[output] FAIL " << label << ": " << what << "." << key << " is "
              << holder[key].GetUint64() << ", expected " << expected
              << std::endl;
    return false;
  }
  return true;
}

// Read the three preference counters of a "soft_skill_preference"
// member; returns false when the member or a counter is missing.
bool read_pref(const std::string& label,
               const std::string& where,
               const rapidjson::Value& holder,
               std::uint64_t& eligible,
               std::uint64_t& satisfied,
               std::uint64_t& unsatisfied) {
  if (!holder.HasMember("soft_skill_preference")) {
    std::cerr << "[output] FAIL " << label << ": " << where
              << " holds no soft_skill_preference" << std::endl;
    return false;
  }
  const auto& pref = holder["soft_skill_preference"];
  if (!pref.IsObject() || !pref.HasMember("eligible") ||
      !pref.HasMember("satisfied") || !pref.HasMember("unsatisfied")) {
    std::cerr << "[output] FAIL " << label << ": " << where
              << " soft_skill_preference lacks counters" << std::endl;
    return false;
  }
  eligible = pref["eligible"].GetUint64();
  satisfied = pref["satisfied"].GetUint64();
  unsatisfied = pref["unsatisfied"].GetUint64();
  return true;
}

// One fixture run: parse, solve, serialize, assert on the document.
struct FixtureCase {
  std::string file;
  std::string label;
  // Expected summary preference counters; nullopt when the output
  // must hold no soft_skill_preference at all.
  std::optional<std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>>
    summary_pref;
};

bool check_fixture(const FixtureCase& fc) {
  const std::string& label = fc.label;
  bool ok = true;

  const auto json = read_fixture(fc.file);
  if (json.empty()) {
    std::cerr << "[output] FAIL " << label << ": fixture not found"
              << std::endl;
    return false;
  }

  vroom::Input input;
  vroom::Solution sol = [&] {
    vroom::io::parse(input, json, false);
    return input.solve(vroom::DEFAULT_EXPLORATION_LEVEL, 1);
  }();

  const bool report_distances = input.report_distances();
  const auto doc = vroom::io::to_json(sol, report_distances);

  ok = expect_true(label, "root is not an object", doc.IsObject()) &&
       expect_true(label, "output code is not 0",
                   doc.HasMember("code") && doc["code"].IsInt() &&
                     doc["code"].GetInt() == 0) &&
       expect_true(label, "missing summary", doc.HasMember("summary") &&
                                              doc["summary"].IsObject()) &&
       expect_true(label, "missing routes array",
                   doc.HasMember("routes") && doc["routes"].IsArray()) &&
       expect_true(label, "missing unassigned array",
                   doc.HasMember("unassigned") && doc["unassigned"].IsArray());
  if (!ok) {
    return false;
  }

  const auto& json_summary = doc["summary"];
  const auto& json_routes = doc["routes"];
  const auto& json_unassigned = doc["unassigned"];

  ok = expect_true(label, "routes count mismatch",
                   json_routes.Size() == sol.routes.size()) &&
       expect_true(label, "summary routes count mismatch",
                   json_summary.HasMember("routes") &&
                     json_summary["routes"].IsUint() &&
                     json_summary["routes"].GetUint() == sol.summary.routes) &&
       expect_true(label, "unassigned count mismatch",
                   json_unassigned.Size() == sol.unassigned.size() &&
                     json_summary.HasMember("unassigned") &&
                     json_summary["unassigned"].IsUint() &&
                     json_summary["unassigned"].GetUint() ==
                       sol.summary.unassigned);

  // Standard economics fields unchanged: JSON mirrors the in-memory
  // solution and the distance key appears exactly when distance data
  // was reported.
  ok = ok && expect_int(label, "summary", json_summary, "cost",
                        sol.summary.cost) &&
       expect_int(label, "summary", json_summary, "duration",
                  sol.summary.duration) &&
       expect_int(label, "summary", json_summary, "service",
                  sol.summary.service) &&
       expect_int(label, "summary", json_summary, "waiting_time",
                  sol.summary.waiting_time);
  if (report_distances) {
    ok = ok && expect_int(label, "summary", json_summary, "distance",
                          sol.summary.distance);
  } else {
    ok = ok && expect_true(label, "distance reported without distance data",
                           !json_summary.HasMember("distance"));
  }

  std::uint64_t sum_eligible = 0;
  std::uint64_t sum_satisfied = 0;
  std::uint64_t sum_unsatisfied = 0;

  for (rapidjson::SizeType i = 0; i < json_routes.Size(); ++i) {
    const auto& json_route = json_routes[i];
    const auto& route = sol.routes[i];
    const auto where = "route " + std::to_string(route.vehicle);

    ok = ok && expect_unsigned(label, where, json_route, "vehicle",
                               route.vehicle) &&
           expect_int(label, where, json_route, "cost", route.cost) &&
           expect_int(label, where, json_route, "duration", route.duration) &&
           expect_int(label, where, json_route, "service", route.service) &&
           expect_int(label, where, json_route, "waiting_time",
                      route.waiting_time);
    if (report_distances) {
      ok = ok && expect_int(label, where, json_route, "distance",
                            route.distance);
    }

    const bool route_pref = json_route.HasMember("soft_skill_preference");
    ok = ok && expect_true(
             label,
             "JSON route/summary preference presence mismatch",
             route_pref == json_summary.HasMember("soft_skill_preference"));

    if (route_pref) {
      std::uint64_t eligible = 0;
      std::uint64_t satisfied = 0;
      std::uint64_t unsatisfied = 0;
      ok = ok && read_pref(label, where, json_route, eligible, satisfied,
                           unsatisfied);
      ok = ok && expect_true(label,
                             where + ": eligible != satisfied + unsatisfied",
                             eligible == satisfied + unsatisfied);
      if (route.soft_skill_preference.has_value()) {
        const auto& mem = route.soft_skill_preference.value();
        ok = ok && expect_true(
                 label, where + ": JSON counters differ from the solution",
                 eligible == mem.eligible && satisfied == mem.satisfied &&
                   unsatisfied == mem.unsatisfied);
      }
      sum_eligible += eligible;
      sum_satisfied += satisfied;
      sum_unsatisfied += unsatisfied;
    }
  }

  if (fc.summary_pref.has_value()) {
    const auto& [eligible, satisfied, unsatisfied] = fc.summary_pref.value();
    std::uint64_t json_eligible = 0;
    std::uint64_t json_satisfied = 0;
    std::uint64_t json_unsatisfied = 0;
    ok = ok && read_pref(label, "summary", json_summary, json_eligible,
                         json_satisfied, json_unsatisfied);
    ok = ok && expect_true(label,
                           "summary preference counters differ from expected",
                           json_eligible == eligible &&
                             json_satisfied == satisfied &&
                             json_unsatisfied == unsatisfied);
    ok = ok && expect_true(
             label, "summary preference totals differ from route sums",
             json_eligible == sum_eligible &&
               json_satisfied == sum_satisfied &&
               json_unsatisfied == sum_unsatisfied);
  } else {
    ok = ok &&
         expect_true(label, "baseline output leaks soft_skill_preference",
                     !json_summary.HasMember("soft_skill_preference"));
    for (rapidjson::SizeType i = 0; i < json_routes.Size(); ++i) {
      ok = ok && expect_true(label, "baseline route leaks preference",
                             !json_routes[i].HasMember(
                               "soft_skill_preference"));
    }
  }

  // Unassigned payload shape (fixture 05 keeps its job there).
  if (sol.unassigned.size() == 1) {
    const auto& json_job = json_unassigned[static_cast<rapidjson::SizeType>(0)];
    ok = ok && expect_unsigned(label, "unassigned[0]", json_job, "id",
                               sol.unassigned[0].id);
  }

  return ok;
}

} // namespace

bool run_output_tests() {
  std::size_t total = 0;
  std::size_t failures = 0;
  const auto check = [&](bool passed) {
    ++total;
    if (!passed) {
      ++failures;
    }
  };

  const std::vector<FixtureCase> cases = {
    {"vroom_soft_skill_fixtures/00_baseline_upstream.json", "00 baseline",
     std::nullopt},
    {"vroom_soft_skill_fixtures/01_all_vs_any.json", "01 all vs any",
     std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>{1, 1, 0}},
    {"vroom_soft_skill_fixtures/02_any_override.json", "02 any override",
     std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>{1, 1, 0}},
    {"vroom_soft_skill_fixtures/03_shipment_override.json",
     "03 shipment override",
     std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>{1, 1, 0}},
    {"vroom_soft_skill_fixtures/04_hard_skills_override_soft.json",
     "04 hard skills override soft",
     std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>{1, 1, 0}},
    {"vroom_soft_skill_fixtures/05_unassigned_capacity.json",
     "05 unassigned capacity",
     std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>{0, 0, 0}},
    {"vroom_soft_skill_fixtures/06_empty_skills_neutral.json",
     "06 empty skills neutral",
     std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>{0, 0, 0}},
  };

  for (const auto& fc : cases) {
    check(check_fixture(fc));
  }

  // Exception serialization keeps the documented shape.
  {
    const std::string label = "exception json";
    const auto exc = vroom::InputException("soft skills probe failure");
    const auto doc = vroom::io::to_json(exc);
    bool ok = doc.IsObject() && doc.HasMember("code") &&
              doc["code"].IsUint() && doc["code"].GetUint() == 2 &&
              doc.HasMember("error") && doc["error"].IsString();
    ok = ok && expect_true(label, "exception payload malformed", ok);
    check(ok);
  }

  std::cout << (total - failures) << "/" << total
            << " output tests passed" << std::endl;
  return failures == 0 && total > 0;
}
