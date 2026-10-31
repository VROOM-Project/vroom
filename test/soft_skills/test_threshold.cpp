// Soft-skills threshold predicate tests (CSV-driven).
//
// Reads vroom_soft_skill_fixtures/threshold_cases.csv and checks
// threshold_passes() in BOTH mode / REJECT policy / distance-data
// available against the expected both_thresholds_pass column.
// Returns true iff every case passes.

#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "structures/typedefs.h"
#include "structures/vroom/soft_skills_options.h"
#include "utils/soft_skills.h"

namespace {

std::string fixture_path(const std::string& name) {
  std::ifstream probe(name);
  if (probe.good()) {
    return name;
  }
  return "../../" + name;
}

void strip_cr(std::string& line) {
  if (!line.empty() && line.back() == '\r') {
    line.pop_back();
  }
}

std::vector<std::string> split_commas(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

} // namespace

bool run_threshold_tests() {
  constexpr const char* fixture =
    "vroom_soft_skill_fixtures/threshold_cases.csv";

  std::ifstream in(fixture_path(fixture));
  if (!in.is_open()) {
    std::cerr << "[threshold] cannot open " << fixture << std::endl;
    return false;
  }

  std::string line;
  std::getline(in, line); // CSV header row

  std::size_t total = 0;
  std::size_t failures = 0;

  while (std::getline(in, line)) {
    strip_cr(line);
    if (line.empty()) {
      continue;
    }

    auto fields = split_commas(line);
    if (fields.size() != 8) {
      std::cerr << "[threshold] malformed row: " << line << std::endl;
      ++failures;
      continue;
    }

    const std::string& case_id = fields[0];
    const double distance_factor = std::stod(fields[1]);
    const double cost_factor = std::stod(fields[2]);
    const vroom::Distance reference_distance = std::stoll(fields[3]);
    const vroom::Distance preferred_distance = std::stoll(fields[4]);
    const vroom::Cost reference_cost = std::stoll(fields[5]);
    const vroom::Cost preferred_cost = std::stoll(fields[6]);
    const bool expected = fields[7] == "True";

    const bool actual = vroom::threshold_passes(preferred_cost,
                                                reference_cost,
                                                cost_factor,
                                                preferred_distance,
                                                reference_distance,
                                                distance_factor,
                                                vroom::ThresholdMode::BOTH,
                                                vroom::MissingDistancePolicy::REJECT,
                                                true);

    ++total;
    if (actual != expected) {
      std::cerr << "[threshold] FAIL " << case_id << ": expected "
                << std::boolalpha << expected << ", got " << actual
                << std::endl;
      ++failures;
    }
  }

  std::cout << (total - failures) << "/" << total
            << " threshold tests passed" << std::endl;
  return failures == 0 && total > 0;
}
