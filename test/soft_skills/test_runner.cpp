// This file is part of the soft-vehicle-skills test harness.
//
// Main entry: runs every test suite and returns 0 on pass.
// Scaffold only — suites are empty placeholders until their tasks land.

#include <iostream>

void run_matching_tests();
void run_threshold_tests();
void run_parsing_tests();
void run_solver_tests();
void run_output_tests();
void run_thread_safety_tests();

int main() {
  run_matching_tests();
  run_threshold_tests();
  run_parsing_tests();
  run_solver_tests();
  run_output_tests();
  run_thread_safety_tests();

  std::cout << "0 tests run" << std::endl;
  return 0;
}
