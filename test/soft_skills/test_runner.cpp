// This file is part of the soft-vehicle-skills test harness.
//
// Main entry: runs every implemented test suite and returns 0 on pass.
// Flags: --matching or --threshold run only that suite; default runs
// both. Remaining suites are placeholders until their tasks land.

#include <cstring>
#include <iostream>

bool run_matching_tests();
bool run_threshold_tests();
void run_parsing_tests();
void run_solver_tests();
void run_output_tests();
void run_thread_safety_tests();



int main(int argc, char** argv) {
  bool run_matching = false;
  bool run_threshold = false;

  if (argc == 1) {
    run_matching = true;
    run_threshold = true;
  } else if (argc == 2 && std::strcmp(argv[1], "--matching") == 0) {
    run_matching = true;
  } else if (argc == 2 && std::strcmp(argv[1], "--threshold") == 0) {
    run_threshold = true;
  } else {
    std::cout << "usage: test_runner [--matching | --threshold]"
              << std::endl;
    return 2;
  }

  bool all_passed = true;

  if (run_matching) {
    all_passed = run_matching_tests() && all_passed;
  }
  if (run_threshold) {
    all_passed = run_threshold_tests() && all_passed;
  }

  return all_passed ? 0 : 1;
}
