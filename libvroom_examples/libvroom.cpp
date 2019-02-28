#include <iostream>

#include "routing/routed_wrapper.h"
#include "structures/vroom/input/input.h"
#include "structures/vroom/job.h"
#include "structures/vroom/vehicle.h"
#include "utils/exception.h"

void log_solution(const vroom::Solution& sol, bool geometry) {
  std::cout << "Total cost: " << sol.summary.cost << std::endl;
  std::cout << "Unassigned: " << sol.summary.unassigned << std::endl;

  // Log unassigned jobs if any.
  std::cout << "Unassigned job ids: ";
  for (const auto& j : sol.unassigned) {
    std::cout << j.id << ", ";
  }
  std::cout << std::endl;

  // Describe routes in solution.
  for (const auto& route : sol.routes) {
    std::cout << "Steps for vehicle " << route.vehicle
              << " (cost: " << route.cost;
    std::cout << " - duration: " << route.duration;
    std::cout << " - service: " << route.service;
    if (geometry) {
      std::cout << " - distance: " << route.distance;
    }

    std::cout << ")" << std::endl;

    // Describe all route steps.
    for (const auto& step : route.steps) {
      std::string step_type;
      switch (step.type) {
      case vroom::STEP_TYPE::START:
        step_type = "Start";
        break;
      case vroom::STEP_TYPE::END:
        step_type = "End";
        break;
      case vroom::STEP_TYPE::JOB:
        step_type = "Job";
        break;
      }
      std::cout << step_type;

      // Add job ids.
      if (step.type == vroom::STEP_TYPE::JOB) {
        std::cout << " " << step.job;
      }

      // Add location if known.
      if (step.location.has_coordinates()) {
        std::cout << " - " << step.location.lon() << ";" << step.location.lat();
      }

      std::cout << " - arrival: " << step.arrival;
      std::cout << " - duration: " << step.duration;
      std::cout << " - service: " << step.service;

      // Add extra step info if geometry is required.
      if (geometry) {
        std::cout << " - distance: " << step.distance;
      }
      std::cout << std::endl;
    }
  }
}

void run_example_with_osrm() {
  bool GEOMETRY = true;

  // Set OSRM host and port.
  auto routing_wrapper =
    std::make_unique<vroom::routing::RoutedWrapper>("car",
                                                    vroom::Server("localhost",
                                                                  "5000"));

  vroom::Input problem_instance;
  problem_instance.set_routing(std::move(routing_wrapper));
  problem_instance.set_geometry(GEOMETRY); // Query for route geometry
                                           // after solving.

  // Create one-dimension capacity restrictions to model the situation
  // where one vehicle can handle 4 jobs.
  vroom::Amount vehicle_capacity(1);
  vroom::TimeWindow vehicle_tw(28800, 43200); // Working hours.
  vroom::Amount job_amount(1);
  vroom::Duration service = 5 * 60; // 5 minutes
  vehicle_capacity[0] = 4;
  job_amount[0] = 1;

  // Define vehicles (use boost::none for no start or no end).
  vroom::Location depot(vroom::Coordinates({{2.35044, 48.71764}}));
  vroom::Vehicle v1(1,                // id
                    depot,            // start
                    depot,            // end
                    vehicle_capacity, // capacity
                    {1, 14},          // skills
                    vehicle_tw);      // time window
  problem_instance.add_vehicle(v1);

  vroom::Vehicle v2(2,                // id
                    depot,            // start
                    depot,            // end
                    vehicle_capacity, // capacity
                    {2, 14},          // skills
                    vehicle_tw);      // time window
  problem_instance.add_vehicle(v2);

  // Job to be done between 9 and 10 AM.
  std::vector<vroom::TimeWindow> job_1_tws({{32400, 36000}});

  // Set jobs id, location, service time, amount and required skills.
  // Last three can be omitted if no constraints are required.
  std::vector<vroom::Job> jobs;
  jobs.push_back(vroom::Job(1,
                            vroom::Coordinates({{1.98935, 48.701}}),
                            service,
                            job_amount,
                            {1},
                            job_1_tws));
  jobs.push_back(vroom::Job(2,
                            vroom::Coordinates({{2.03655, 48.61128}}),
                            service,
                            job_amount,
                            {1}));
  jobs.push_back(vroom::Job(3,
                            vroom::Coordinates({{2.39719, 49.07611}}),
                            service,
                            job_amount,
                            {2}));
  jobs.push_back(vroom::Job(4,
                            vroom::Coordinates({{2.41808, 49.22619}}),
                            service,
                            job_amount,
                            {2}));
  jobs.push_back(vroom::Job(5,
                            vroom::Coordinates({{2.28325, 48.5958}}),
                            service,
                            job_amount,
                            {14}));
  jobs.push_back(vroom::Job(6,
                            vroom::Coordinates({{2.89357, 48.90736}}),
                            service,
                            job_amount,
                            {14}));

  for (const auto& j : jobs) {
    problem_instance.add_job(j);
  }

  // Skills definitions set the following constraints:
  // - jobs 1 and 2 can only be served by vehicle 1
  // - jobs 3 and 4 can only be served by vehicle 2
  // - jobs 5 and 6 can be served by either one of the vehicles

  // Solve!
  try {
    auto sol = problem_instance.solve(5,  // Exploration level.
                                      4); // Use 4 threads.

    log_solution(sol, GEOMETRY);
  } catch (const vroom::Exception& e) {
    std::cerr << "[Error] " << e.message << std::endl;
  }
}

void run_example_with_custom_matrix() {
  bool GEOMETRY = false;

  vroom::Input problem_instance;

  // Define custom matrix and bypass OSRM call.
  vroom::Matrix<vroom::Cost> matrix_input(
    {{0, 2713, 2218, 4317, 5698, 2191, 3528},
     {2876, 0, 1109, 5198, 6361, 2963, 5385},
     {2359, 1082, 0, 5797, 7178, 1883, 5008},
     {4097, 5228, 5584, 0, 2236, 5511, 3669},
     {5472, 6432, 6959, 2232, 0, 6886, 4581},
     {2083, 2954, 1887, 5736, 7117, 0, 4593},
     {3679, 5526, 5166, 3506, 4471, 4631, 0}});
  problem_instance.set_matrix(std::move(matrix_input));

  // Create one-dimension capacity restrictions to model the situation
  // where one vehicle can handle 4 jobs.
  vroom::Amount vehicle_capacity(1);
  vroom::TimeWindow vehicle_tw(28800, 43200); // Working hours.
  vroom::Amount job_amount(1);
  vroom::Duration service = 5 * 60; // 5 minutes
  vehicle_capacity[0] = 4;
  job_amount[0] = 1;

  // Define vehicles (use boost::none for no start or no end).
  vroom::Location depot(0); // index in the provided matrix.

  vroom::Vehicle v1(1,                // id
                    depot,            // start
                    depot,            // end
                    vehicle_capacity, // capacity
                    {1, 14},          // skills
                    vehicle_tw);      // time window
  problem_instance.add_vehicle(v1);

  vroom::Vehicle v2(2,                // id
                    depot,            // start
                    depot,            // end
                    vehicle_capacity, // capacity
                    {2, 14},          // skills
                    vehicle_tw);      // time window
  problem_instance.add_vehicle(v2);

  // Job to be done between 9 and 10 AM.
  std::vector<vroom::TimeWindow> job_1_tws({{32400, 36000}});

  // Set jobs id, index of location in the matrix (coordinates are
  // optional), amount, required skills and time windows. Last three
  // can be omitted if no constraints are required.
  std::vector<vroom::Job> jobs;
  jobs.push_back(vroom::Job(1, 1, service, job_amount, {1}, job_1_tws));
  jobs.push_back(vroom::Job(2, 2, service, job_amount, {1}));
  jobs.push_back(vroom::Job(3, 3, service, job_amount, {2}));
  jobs.push_back(vroom::Job(4, 4, service, job_amount, {2}));
  jobs.push_back(vroom::Job(5, 5, service, job_amount, {14}));
  jobs.push_back(vroom::Job(6, 6, service, job_amount, {14}));

  for (const auto& j : jobs) {
    problem_instance.add_job(j);
  }

  // Skills definitions set the following constraints:
  // - jobs 1 and 2 can only be served by vehicle 1
  // - jobs 3 and 4 can only be served by vehicle 2
  // - jobs 5 and 6 can be served by either one of the vehicles

  // Solve!
  try {
    auto sol = problem_instance.solve(5,  // Exploration level.
                                      4); // Use 4 threads.

    log_solution(sol, GEOMETRY);
  } catch (const vroom::Exception& e) {
    std::cerr << "[Error] " << e.message << std::endl;
  }
}

int main() {
  run_example_with_osrm();
  // run_example_with_custom_matrix();

  return 0;
}
