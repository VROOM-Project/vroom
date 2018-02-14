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

int main() {
  // Create wrapper for OSRM queries.
  auto routing_wrapper =
    std::make_unique<routed_wrapper>("localhost", // OSRM server
                                     "5000",      // OSRM port
                                     "car"        // Profile
                                     );

  input problem_instance(std::move(routing_wrapper),
                         true); // Query for route geometry after solving.

  matrix<cost_t> matrix_input(4);
  matrix_input[0][0] = 0;
  matrix_input[0][1] = 0;
  matrix_input[0][2] = 775;
  matrix_input[0][3] = 1340;

  matrix_input[1][0] = 0;
  matrix_input[1][1] = 0;
  matrix_input[1][2] = 775;
  matrix_input[1][3] = 1340;

  matrix_input[2][0] = 746;
  matrix_input[2][1] = 746;
  matrix_input[2][2] = 0;
  matrix_input[2][3] = 1635;

  matrix_input[3][0] = 1507;
  matrix_input[3][1] = 1507;
  matrix_input[3][2] = 1593;
  matrix_input[3][3] = 0;

  problem_instance._matrix = matrix_input;

  // Define vehicle with id, start and end (use boost::none for no
  // start or no end).
  vehicle_t v(0, location_t({2.3526, 48.8604}), location_t({2.3526, 48.8604}));

  problem_instance.add_vehicle(v);

  // Set jobs ids and locations.
  job_t j1(1, coords_t({2.3691, 48.8532}));
  job_t j2(2, coords_t({2.2911, 48.8566}));

  problem_instance.add_job(j1);
  problem_instance.add_job(j2);

  // Log level.
  boost::log::core::get()->set_filter(boost::log::trivial::severity >=
                                      boost::log::trivial::error);

  // Solve!
  try {
    auto sol = problem_instance.solve(2); // Use 2 threads.

    std::cout << "Cost: " << sol.summary.cost << std::endl;

    // Describe routes in solution.
    for (const auto& route : sol.routes) {
      std::cout << "Steps for vehicle " << route.vehicle << std::endl;

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
          std::cout << " - " << step.location.lon() << ";"
                    << step.location.lat();
        }
        std::cout << std::endl;
      }
    }
  } catch (const custom_exception& e) {
    std::cerr << "[Error] " << e.get_message() << std::endl;
  }

  return 0;
}
