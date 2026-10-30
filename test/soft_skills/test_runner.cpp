// This file is part of the soft-vehicle-skills test harness.
//
// Main entry: runs every implemented test suite and returns 0 on pass.
// A single --matching/--threshold/--parsing/--solver/--output/
// --thread-safety flag restricts the run to that suite; no flag runs
// all six.

#include <array>
#include <cstring>
#include <iostream>

bool run_matching_tests();
bool run_threshold_tests();
bool run_parsing_tests();
bool run_solver_tests();
bool run_output_tests();
bool run_thread_safety_tests();

namespace {

struct Suite {
  const char* flag;
  const char* name;
  bool (*run)();
};

constexpr std::array<Suite, 6> suites = {{
  {"--matching", "matching", run_matching_tests},
  {"--threshold", "threshold", run_threshold_tests},
  {"--parsing", "parsing", run_parsing_tests},
  {"--solver", "solver", run_solver_tests},
  {"--output", "output", run_output_tests},
  {"--thread-safety", "thread-safety", run_thread_safety_tests},
}};

} // namespace

int main(int argc, char** argv) {
  const Suite* selected = nullptr;

  if (argc > 2) {
    std::cout << "usage: test_runner [--matching | --threshold | --parsing | "
                 "--solver | --output | --thread-safety]"
              << std::endl;
    return 2;
  }

  if (argc == 2) {
    for (const auto& suite : suites) {
      if (std::strcmp(argv[1], suite.flag) == 0) {
        selected = &suite;
        break;
      }
    }
    if (selected == nullptr) {
      std::cout << "usage: test_runner [--matching | --threshold | --parsing | "
                   "--solver | --output | --thread-safety]"
                << std::endl;
      return 2;
    }
  }

  bool all_passed = true;

  for (const auto& suite : suites) {
    if (selected != nullptr && selected != &suite) {
      continue;
    }
    all_passed = suite.run() && all_passed;
  }

  return all_passed ? 0 : 1;
}
