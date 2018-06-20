#include <iostream>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>

#include "routing/routed_wrapper.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/job.h"
#include "structures/vroom/vehicle.h"
#include "utils/exceptions.h"

void log_solution(const solution& sol, bool geometry) {
  std::cout << "Total cost: " << sol.summary.cost << std::endl;
  std::cout << "Unassigned: " << sol.summary.unassigned << std::endl;

  // Log unassigned jobs if any.
  std::cout << "Unassigned job ids: [";
  for (const auto& j : sol.unassigned) {
    std::cout << j.id << ", ";
  }
  std::cout << "]" << std::endl;

  // Describe routes in solution.
  for (const auto& route : sol.routes) {
    std::cout << "Steps for vehicle " << route.vehicle
              << " (cost: " << route.cost;
    if (geometry) {
      std::cout << " - duration: " << route.duration;
      std::cout << " - service: " << route.service;
      std::cout << " - distance: " << route.distance;
    }

    std::cout << ")" << std::endl;

    // Describe all route steps.
    for (const auto& step : route.steps) {
      std::string step_type;
      switch (step.type) {
      case TYPE::START:
        step_type = "Start";
        break;
      case TYPE::END:
        step_type = "End";
        break;
      case TYPE::JOB:
        step_type = "Job";
        break;
      }
      std::cout << step_type;

      // Add job ids.
      if (step.type == TYPE::JOB) {
        std::cout << " " << step.job;
      }

      // Add location if known.
      if (step.location.has_coordinates()) {
        std::cout << " - " << step.location.lon() << ";" << step.location.lat();
      }

      // Add extra step info if geometry is required.
      if (geometry) {
        std::cout << " - arrival: " << step.arrival;
        std::cout << " - duration: " << step.duration;
        std::cout << " - service: " << step.service;
        std::cout << " - distance: " << step.distance;
      }
      std::cout << std::endl;
    }
  }
}

std::unique_ptr<routed_wrapper> routing_wrapper() {
  // Create a wrapper for OSRM queries.
  return std::make_unique<routed_wrapper>("localhost", // OSRM server
                                          "5000",      // OSRM port
                                          "car"        // Profile
                                          );
}

void run_example_with_osrm() {
  bool GEOMETRY = true;

  input problem_instance(std::move(routing_wrapper()),
                         GEOMETRY); // Query for route geometry after solving.

  // Create one-dimension capacity restrictions to model the situation
  // where one vehicle can handle 4 jobs.
  amount_t vehicle_capacity(1);
  amount_t job_amount(1);
  duration_t service = 5 * 60; // 5 minutes
  vehicle_capacity[0] = 4;
  job_amount[0] = 1;

  // Define vehicles (use boost::none for no start or no end).
  location_t depot({2.35044, 48.71764});
  vehicle_t v1(1,                // id
               depot,            // start
               depot,            // end
               vehicle_capacity, // capacity
               {1, 14});         // skills
  problem_instance.add_vehicle(v1);

  vehicle_t v2(2,                // id
               depot,            // start
               depot,            // end
               vehicle_capacity, // capacity
               {2, 14});         // skills
  problem_instance.add_vehicle(v2);

  // Set jobs id, location, service time, amount and required skills.
  // Last three can be omitted if no constraints are required.
  std::vector<job_t> jobs;
  jobs.push_back(
    job_t(1, coords_t({1.98935, 48.701}), service, job_amount, {1}));
  jobs.push_back(
    job_t(2, coords_t({2.03655, 48.61128}), service, job_amount, {1}));
  jobs.push_back(
    job_t(3, coords_t({2.39719, 49.07611}), service, job_amount, {2}));
  jobs.push_back(
    job_t(4, coords_t({2.41808, 49.22619}), service, job_amount, {2}));
  jobs.push_back(
    job_t(5, coords_t({2.28325, 48.5958}), service, job_amount, {14}));
  jobs.push_back(
    job_t(6, coords_t({2.89357, 48.90736}), service, job_amount, {14}));

  for (const auto& j : jobs) {
    problem_instance.add_job(j);
  }

  // Skills definitions set the following constraints:
  // - jobs 1 and 2 can only be served by vehicle 1
  // - jobs 3 and 4 can only be served by vehicle 2
  // - jobs 5 and 6 can be served by either one of the vehicles

  // Solve!
  try {
    auto sol = problem_instance.solve(1,  // Exploration level.
                                      4); // Use 4 threads.

    log_solution(sol, GEOMETRY);
  } catch (const custom_exception& e) {
    std::cerr << "[Error] " << e.get_message() << std::endl;
  }
}

void run_example_with_custom_matrix() {
  bool GEOMETRY = false;

  input problem_instance(std::move(routing_wrapper()),
                         GEOMETRY); // Query for route geometry after solving.

  // Define custom matrix and bypass OSRM call.
  matrix<cost_t> matrix_input({{0, 2713, 2218, 4317, 5698, 2191, 3528},
                               {2876, 0, 1109, 5198, 6361, 2963, 5385},
                               {2359, 1082, 0, 5797, 7178, 1883, 5008},
                               {4097, 5228, 5584, 0, 2236, 5511, 3669},
                               {5472, 6432, 6959, 2232, 0, 6886, 4581},
                               {2083, 2954, 1887, 5736, 7117, 0, 4593},
                               {3679, 5526, 5166, 3506, 4471, 4631, 0}});
  problem_instance.set_matrix(std::move(matrix_input));

  // Create one-dimension capacity restrictions to model the situation
  // where one vehicle can handle 4 jobs.
  amount_t vehicle_capacity(1);
  amount_t job_amount(1);
  duration_t service = 5 * 60; // 5 minutes
  vehicle_capacity[0] = 4;
  job_amount[0] = 1;

  // Define vehicles (use boost::none for no start or no end).
  location_t depot(0); // index in the provided matrix.

  vehicle_t v1(1,                // id
               depot,            // start
               depot,            // end
               vehicle_capacity, // capacity
               {1, 14});         // skills
  problem_instance.add_vehicle(v1);

  vehicle_t v2(2,                // id
               depot,            // start
               depot,            // end
               vehicle_capacity, // capacity
               {2, 14});         // skills
  problem_instance.add_vehicle(v2);

  // Set jobs id, index of location in the matrix (coordinates are
  // optional), amount and required skills. Last two can be omitted if
  // no constraints are required.
  std::vector<job_t> jobs;
  jobs.push_back(job_t(1, 1, service, job_amount, {1}));
  jobs.push_back(job_t(2, 2, service, job_amount, {1}));
  jobs.push_back(job_t(3, 3, service, job_amount, {2}));
  jobs.push_back(job_t(4, 4, service, job_amount, {2}));
  jobs.push_back(job_t(5, 5, service, job_amount, {14}));
  jobs.push_back(job_t(6, 6, service, job_amount, {14}));

  for (const auto& j : jobs) {
    problem_instance.add_job(j);
  }

  // Skills definitions set the following constraints:
  // - jobs 1 and 2 can only be served by vehicle 1
  // - jobs 3 and 4 can only be served by vehicle 2
  // - jobs 5 and 6 can be served by either one of the vehicles

  // Solve!
  try {
    auto sol = problem_instance.solve(1,  // Exploration level.
                                      4); // Use 4 threads.

    log_solution(sol, GEOMETRY);
  } catch (const custom_exception& e) {
    std::cerr << "[Error] " << e.get_message() << std::endl;
  }
}

int main() {
  // Log level.
  boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                      boost::log::trivial::error);

  run_example_with_osrm();
  // run_example_with_custom_matrix();

  return 0;
}
