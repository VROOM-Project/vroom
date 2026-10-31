// Soft-skills matching predicate tests (CSV-driven).
//
// Reads vroom_soft_skill_fixtures/matching_cases.csv and checks
// soft_skill_match() against the expected is_preferred column.
// Returns true iff every case passes.

#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "structures/typedefs.h"
#include "utils/soft_skills.h"

namespace {

// Locate the fixture file relative to the repo root (default run
// location) or to the test binary (run from test/soft_skills).
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

vroom::SoftSkills parse_skills(const std::string& field) {
  vroom::SoftSkills skills;
  std::stringstream ss(field);
  std::string token;
  while (std::getline(ss, token, ';')) {
    if (token.empty()) {
      continue;
    }
    skills.insert(static_cast<vroom::Skill>(std::stoul(token)));
  }
  return skills;
}

} // namespace

bool run_matching_tests() {
  constexpr const char* fixture =
    "vroom_soft_skill_fixtures/matching_cases.csv";

  std::ifstream in(fixture_path(fixture));
  if (!in.is_open()) {
    std::cerr << "[matching] cannot open " << fixture << std::endl;
    return false;
  }

  std::string line;
  std::getline(in, line); // header

  std::size_t total = 0;
  std::size_t failures = 0;

  while (std::getline(in, line)) {
    strip_cr(line);
    if (line.empty()) {
      continue;
    }

    auto fields = split_commas(line);
    if (fields.size() != 5) {
      std::cerr << "[matching] malformed row: " << line << std::endl;
      ++failures;
      continue;
    }

    const std::string& case_id = fields[0];
    const std::string& mode = fields[1];
    const auto task_skills = parse_skills(fields[2]);
    const auto vehicle_skills = parse_skills(fields[3]);

    vroom::SoftSkillMatchMode match_mode;
    if (mode == "all") {
      match_mode = vroom::SoftSkillMatchMode::ALL;
    } else if (mode == "any") {
      match_mode = vroom::SoftSkillMatchMode::ANY;
    } else {
      std::cerr << "[matching] unknown mode '" << mode << "' in case "
                << case_id << std::endl;
      ++failures;
      continue;
    }

    const bool expected = fields[4] == "True";
    const bool actual =
      vroom::soft_skill_match(task_skills, vehicle_skills, match_mode);

    ++total;
    if (actual != expected) {
      std::cerr << "[matching] FAIL " << case_id << ": expected "
                << std::boolalpha << expected << ", got " << actual
                << std::endl;
      ++failures;
    }
  }

  std::cout << (total - failures) << "/" << total
            << " matching tests passed" << std::endl;
  return failures == 0 && total > 0;
}
