#include <iostream>

#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/console.hpp>

#include "../src/routing/routed_wrapper.h"
#include "../src/structures/vroom/input/input.h"
#include "../src/structures/vroom/job.h"
#include "../src/structures/vroom/vehicle.h"
#include "../src/utils/exceptions.h"

void log_solution(const solution& sol) {
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
              << " (cost: " << route.cost << ")" << std::endl;

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
      std::cout << std::endl;
    }
  }
}

void run_example_with_osrm() {
  // Create a wrapper for OSRM queries.
  auto routing_wrapper =
    std::make_unique<routed_wrapper>("localhost", // OSRM server
                                     "5000",      // OSRM port
                                     "car"        // Profile
                                     );

  input problem_instance(std::move(routing_wrapper),
                         false); // Query for route geometry after solving.

  // Create one-dimension capacity restrictions to model the situation
  // where one vehicle can handle 4 jobs.
  amount_t vehicle_capacity(1);
  amount_t job_amount(1);
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

  // Set jobs id, amount, required skills and location.
  std::vector<job_t> jobs;
  jobs.push_back(job_t(1, job_amount, {1}, coords_t({1.98935, 48.701})));
  jobs.push_back(job_t(2, job_amount, {1}, coords_t({2.03655, 48.61128})));
  jobs.push_back(job_t(3, job_amount, {2}, coords_t({2.39719, 49.07611})));
  jobs.push_back(job_t(4, job_amount, {2}, coords_t({2.41808, 49.22619})));
  jobs.push_back(job_t(5, job_amount, {14}, coords_t({2.28325, 48.5958})));
  jobs.push_back(job_t(6, job_amount, {14}, coords_t({2.89357, 48.90736})));

  for (const auto& j : jobs) {
    problem_instance.add_job(j);
  }

  // Skills definitions set the following constraints:
  // - jobs 1 and 2 can only be served by vehicle 1
  // - jobs 3 and 4 can only be served by vehicle 2
  // - jobs 5 and 6 can be served by either one of the vehicles

  // Solve!
  try {
    auto sol = problem_instance.solve(2); // Use 2 threads.

    log_solution(sol);
  } catch (const custom_exception& e) {
    std::cerr << "[Error] " << e.get_message() << std::endl;
  }
}

int main() {
  // Log level.
  boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                      boost::log::trivial::error);

  run_example_with_osrm();

  return 0;
}
