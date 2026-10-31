// Soft-skills JSON input parsing validation tests (IS-7).
//
// Checks that every fixture (00-06) parses cleanly through
// vroom::io::parse with the expected option/field defaults, and that
// invalid soft-skills inputs raise InputException (error code 2) with
// descriptive messages: unknown modes, factors below 1.0, endpoint
// soft_skills on shipment legs, distance thresholds without distance
// data, mode overrides on tasks without skills, malformed skill ids
// and unknown option keys.
//
// Returns true iff every check passes.

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "../../include/rapidjson/include/rapidjson/document.h"
#include "../../include/rapidjson/include/rapidjson/stringbuffer.h"
#include "../../include/rapidjson/include/rapidjson/writer.h"

#include "structures/typedefs.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/job.h"
#include "structures/vroom/soft_skills_options.h"
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

std::string read_file(const std::string& name) {
  std::ifstream in(fixture_path(name));
  if (!in.is_open()) {
    return "";
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  return buffer.str();
}

// Compact re-serialization so the targeted mutation needles below
// match no matter how the fixture files are indented.
std::string minify(const std::string& json) {
  rapidjson::Document doc;
  doc.Parse(json.c_str());
  if (doc.HasParseError()) {
    return "";
  }
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  doc.Accept(writer);
  return buffer.GetString();
}

// Replace the first occurrence of needle (throws away nothing if the
// needle is missing: a missing needle is a test-authoring bug, so we
// surface it through the returned flag).
bool replace_once(std::string& json,
                  const std::string& needle,
                  const std::string& repl) {
  const auto pos = json.find(needle);
  if (pos == std::string::npos) {
    return false;
  }
  json.replace(pos, needle.size(), repl);
  return true;
}

// Parse json into a fresh Input. On success fill input and return
// true; on InputException fill error and return false.
bool try_parse(vroom::Input& input,
               const std::string& json,
               std::string& error,
               unsigned& error_code) {
  try {
    vroom::io::parse(input, json, false);
    return true;
  } catch (const vroom::Exception& e) {
    error = e.message;
    error_code = e.error_code;
    return false;
  }
}

// Expect the JSON to parse without any exception.
bool expect_parse_ok(const std::string& label,
                     const std::string& json,
                     bool expect_soft_skills) {
  if (json.empty()) {
    std::cerr << "[parsing] FAIL " << label << ": fixture not found"
              << std::endl;
    return false;
  }
  try {
    vroom::Input input;
    vroom::io::parse(input, json, false);
    if (input.has_soft_skills() != expect_soft_skills) {
      std::cerr << "[parsing] FAIL " << label << ": has_soft_skills() is "
                << std::boolalpha << input.has_soft_skills() << ", expected "
                << expect_soft_skills << std::endl;
      return false;
    }
    return true;
  } catch (const vroom::Exception& e) {
    std::cerr << "[parsing] FAIL " << label << ": unexpected InputException: "
              << e.message << std::endl;
    return false;
  }
}

// Expect the JSON to raise an InputException (code 2) whose message
// contains the given fragment.
bool expect_parse_error(const std::string& label,
                        const std::string& json,
                        const std::string& message_fragment) {
  vroom::Input input;
  std::string error;
  unsigned code = 0;
  if (try_parse(input, json, error, code)) {
    std::cerr << "[parsing] FAIL " << label
              << ": expected rejection, parsed fine instead" << std::endl;
    return false;
  }
  if (code != 2) {
    std::cerr << "[parsing] FAIL " << label << ": expected error code 2, got "
              << code << " (" << error << ")" << std::endl;
    return false;
  }
  if (error.find(message_fragment) == std::string::npos) {
    std::cerr << "[parsing] FAIL " << label << ": message \"" << error
              << "\" does not contain \"" << message_fragment << "\""
              << std::endl;
    return false;
  }
  return true;
}

bool expect_eq(const std::string& label,
               const std::string& what,
               std::size_t actual,
               std::size_t expected) {
  if (actual != expected) {
    std::cerr << "[parsing] FAIL " << label << ": " << what << " is " << actual
              << ", expected " << expected << std::endl;
    return false;
  }
  return true;
}

// Job lookup by id (SINGLE or PICKUP endpoints only).
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

template <class T>
bool expect_mode(const std::string& label,
                 const std::string& what,
                 T actual,
                 T expected) {
  if (actual != expected) {
    std::cerr << "[parsing] FAIL " << label << ": " << what
              << " does not match expected value" << std::endl;
    return false;
  }
  return true;
}

} // namespace

bool run_parsing_tests() {
  std::size_t total = 0;
  std::size_t failures = 0;
  const auto check = [&](bool passed) {
    ++total;
    if (!passed) {
      ++failures;
    }
  };

  // Valid fixture parsing (00-06).
  const std::vector<std::pair<std::string, bool>> fixtures = {
    {"vroom_soft_skill_fixtures/00_baseline_upstream.json", false},
    {"vroom_soft_skill_fixtures/01_all_vs_any.json", true},
    {"vroom_soft_skill_fixtures/02_any_override.json", true},
    {"vroom_soft_skill_fixtures/03_shipment_override.json", true},
    {"vroom_soft_skill_fixtures/04_hard_skills_override_soft.json", true},
    {"vroom_soft_skill_fixtures/05_unassigned_capacity.json", true},
    {"vroom_soft_skill_fixtures/06_empty_skills_neutral.json", true},
  };
  for (std::size_t f = 0; f < fixtures.size(); ++f) {
    const std::string label = "valid fixture " + std::to_string(f);
    check(
          expect_parse_ok(label, read_file(fixtures[f].first), fixtures[f].second));
  }

  // Parsed field values on a few representative fixtures.
  {
    const std::string label = "fixture 00 defaults";
    vroom::Input input;
    bool ok = expect_parse_ok(label, read_file(fixtures[0].first), false);
    const auto& opts = input.get_soft_skills_options();
    ok = ok && expect_mode(label,
                           "default_match_mode",
                           opts.default_match_mode,
                           vroom::SoftSkillMatchMode::ALL);
    ok = ok && expect_mode(label,
                           "threshold_mode",
                           opts.threshold_mode,
                           vroom::ThresholdMode::COST);
    ok = ok && (opts.max_cost_factor == 1.0) && (opts.max_distance_factor == 1.0);
    ok = ok && expect_mode(label,
                           "missing_distance_policy",
                           opts.missing_distance_policy,
                           vroom::MissingDistancePolicy::REJECT);
    check(ok);
  }
  {
    const std::string label = "fixture 01 options and fields";
    vroom::Input input;
    bool ok = true;
    try {
      vroom::io::parse(input, read_file(fixtures[1].first), false);
    } catch (const vroom::Exception& e) {
      std::cerr << "[parsing] FAIL " << label << ": " << e.message << std::endl;
      ok = false;
    }
    if (ok) {
      const auto& opts = input.get_soft_skills_options();
      ok = expect_mode(label,
                       "threshold_mode",
                       opts.threshold_mode,
                       vroom::ThresholdMode::DISTANCE);
      ok = ok && (opts.max_distance_factor == 2.0);
      const auto* v1 = find_vehicle(input, 1);
      const auto* v2 = find_vehicle(input, 2);
      const auto* job = find_job(input, 101);
      ok = ok && v1 != nullptr && v2 != nullptr && job != nullptr;
      if (ok) {
        ok = expect_eq(label, "vehicle 1 soft_skills", v1->soft_skills.size(), 1) &&
             expect_eq(label, "vehicle 2 soft_skills", v2->soft_skills.size(), 2) &&
             expect_eq(label, "job soft_skills", job->soft_skills.size(), 2);
        ok = ok && job->soft_skill_match_mode.has_value() &&
             job->soft_skill_match_mode.value() ==
               vroom::SoftSkillMatchMode::ALL;
      }
    }
    check(ok);
  }
  {
    const std::string label = "fixture 02 job override any";
    vroom::Input input;
    bool ok = true;
    try {
      vroom::io::parse(input, read_file(fixtures[2].first), false);
    } catch (const vroom::Exception& e) {
      std::cerr << "[parsing] FAIL " << label << ": " << e.message << std::endl;
      ok = false;
    }
    if (ok) {
      const auto* job = find_job(input, 101);
      ok = job != nullptr && job->soft_skill_match_mode.has_value() &&
           job->soft_skill_match_mode.value() == vroom::SoftSkillMatchMode::ANY;
      if (!ok) {
        std::cerr << "[parsing] FAIL " << label
                  << ": job soft_skill_match_mode is not any" << std::endl;
      }
    }
    check(ok);
  }
  {
    const std::string label = "fixture 03 shipment-level fields";
    vroom::Input input;
    bool ok = true;
    try {
      vroom::io::parse(input, read_file(fixtures[3].first), false);
    } catch (const vroom::Exception& e) {
      std::cerr << "[parsing] FAIL " << label << ": " << e.message << std::endl;
      ok = false;
    }
    if (ok) {
      const auto& opts = input.get_soft_skills_options();
      ok = expect_mode(label,
                       "default_match_mode",
                       opts.default_match_mode,
                       vroom::SoftSkillMatchMode::ANY);
      const auto* pickup = find_job(input, 201);
      ok = ok && pickup != nullptr;
      if (pickup != nullptr) {
        ok = ok &&
             expect_eq(label, "pickup soft_skills", pickup->soft_skills.size(), 2) &&
             pickup->soft_skill_match_mode.has_value() &&
             pickup->soft_skill_match_mode.value() ==
               vroom::SoftSkillMatchMode::ALL;
      }
    }
    check(ok);
  }
  {
    const std::string label = "fixture 04 both-mode factors";
    vroom::Input input;
    bool ok = true;
    try {
      vroom::io::parse(input, read_file(fixtures[4].first), false);
    } catch (const vroom::Exception& e) {
      std::cerr << "[parsing] FAIL " << label << ": " << e.message << std::endl;
      ok = false;
    }
    if (ok) {
      const auto& opts = input.get_soft_skills_options();
      ok = expect_mode(label,
                       "threshold_mode",
                       opts.threshold_mode,
                       vroom::ThresholdMode::BOTH);
      ok = ok && (opts.max_cost_factor == 1.5) &&
           (opts.max_distance_factor == 2.0);
      const auto* job = find_job(input, 301);
      ok = ok && job != nullptr && expect_eq(label, "job hard skills",
                                             job->skills.size(), 1);
    }
    check(ok);
  }
  {
    const std::string label = "fixture 06 empty skill lists allowed";
    vroom::Input input;
    bool ok = true;
    try {
      vroom::io::parse(input, read_file(fixtures[6].first), false);
    } catch (const vroom::Exception& e) {
      std::cerr << "[parsing] FAIL " << label << ": " << e.message << std::endl;
      ok = false;
    }
    if (ok) {
      const auto* v1 = find_vehicle(input, 1);
      const auto* job = find_job(input, 101);
      ok = ok && v1 != nullptr && job != nullptr;
      if (ok) {
        ok = expect_eq(label, "vehicle 1 soft_skills", v1->soft_skills.size(), 0) &&
             expect_eq(label, "job soft_skills", job->soft_skills.size(), 0);
      }
    }
    check(ok);
  }

  // Invalid inputs: built by targeted edits of the fixture 01 / 03
  // documents. Each edit must hit its needle, otherwise the check
  // fails loudly.
  const std::string base_job = minify(read_file(fixtures[1].first));
  const std::string base_shipment = minify(read_file(fixtures[3].first));

  const auto invalid = [&](const std::string& label,
                           std::string json,
                           const std::string& needle,
                           const std::string& repl,
                           const std::string& fragment) {
    bool edited = true;
    if (!needle.empty()) {
      edited = replace_once(json, needle, repl);
    }
    check(edited && expect_parse_error(label, json, fragment));
  };

  invalid("bad task match mode",
          base_job,
          "\"soft_skill_match_mode\":\"all\"",
          "\"soft_skill_match_mode\":\"prefer\"",
          "Invalid soft_skill_match_mode");
  invalid("bad global default match mode",
          base_job,
          "\"default_match_mode\":\"all\"",
          "\"default_match_mode\":\"some\"",
          "Invalid default_match_mode");
  invalid("bad threshold mode",
          base_job,
          "\"threshold_mode\":\"distance\"",
          "\"threshold_mode\":\"either\"",
          "Invalid threshold_mode");
  invalid("distance factor below 1",
          base_job,
          "\"max_distance_factor\":2.0",
          "\"max_distance_factor\":0.5",
          "max_distance_factor");
  invalid("cost factor below 1",
          base_job,
          "\"max_distance_factor\":2.0",
          "\"max_cost_factor\":0.99,\"max_distance_factor\":2.0",
          "max_cost_factor");
  invalid("override mode with empty skills",
          base_job,
          "\"soft_skills\":[101,202],\"soft_skill_match_mode\":\"all\"",
          "\"soft_skills\":[],\"soft_skill_match_mode\":\"all\"",
          "soft_skills must be non-empty");
  invalid("override mode with missing skills",
          base_job,
          "\"soft_skills\":[101,202],",
          "",
          "soft_skills must be non-empty");
  invalid("malformed soft skill id",
          base_job,
          "\"soft_skills\":[101,202]",
          "\"soft_skills\":[101,-3]",
          "Invalid soft skill");
  invalid("distance threshold without distance data",
          base_job,
          "\"distances\":[[0,1000,2000],[1000,0,1000],[2000,1000,0]],",
          "",
          "distance data required");
  invalid("unknown options.soft_skills key",
          base_job,
          "\"default_match_mode\":\"all\",",
          "\"default_match_mode\":\"all\",\"bogus_key\":true,",
          "Unknown key in options.soft_skills");
  invalid("invalid missing_distance_policy",
          base_job,
          "\"max_distance_factor\":2.0",
          "\"max_distance_factor\":2.0,\"missing_distance_policy\":\"maybe\"",
          "Invalid missing_distance_policy");
  invalid("endpoint soft_skills on shipment pickup",
          base_shipment,
          "\"pickup\":{\"id\":201,",
          "\"pickup\":{\"id\":201,\"soft_skills\":[9],",
          "not allowed on shipment pickup");
  invalid("endpoint match mode on shipment delivery",
          base_shipment,
          "\"delivery\":{\"id\":202,",
          "\"delivery\":{\"id\":202,\"soft_skill_match_mode\":\"any\",",
          "not allowed on shipment delivery");

  // Valid policy value accepted alongside distance threshold.
  {
    const std::string label = "valid skip_distance policy";
    std::string json = base_job;
    const bool edited = replace_once(json,
                                     "\"max_distance_factor\":2.0",
                                     "\"max_distance_factor\":2.0,\"missing_"
                                     "distance_policy\":\"skip_distance\"");
    check(edited && expect_parse_ok(label, json, true));
  }

  std::cout << (total - failures) << "/" << total
            << " parsing tests passed" << std::endl;
  return failures == 0 && total > 0;
}
