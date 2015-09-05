/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015, Julien Coupey

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "logger.h"

logger::logger(const cl_args_t& cl_args):
  _cl_args(cl_args) {}

std::string logger::tour_to_string(const tsp& instance,
                                   const std::list<index_t>& tour,
                                   const timing_t& computing_times) const{
  std::string route_summary;
  unsigned long route_summary_duration = 0;
  if(_cl_args.loader == 0 and _cl_args.geometry){
    // Get route informations summary when using OSRM.
    auto start_route_summary = std::chrono::high_resolution_clock::now();
    route_summary = instance.get_route_summary(tour);
    auto end_route_summary = std::chrono::high_resolution_clock::now();
    route_summary_duration =
      std::chrono::duration_cast<std::chrono::milliseconds>
      (end_route_summary - start_route_summary).count();
  }

  // Execution informations.
  std::string json_log = "{\"computing_times\":{";
  json_log += "\"matrix_loading\":"
    + std::to_string(computing_times.matrix_loading) + ",";
  json_log += "\"route\":{\"heuristic\":"
    + std::to_string(computing_times.heuristic) + ",";
  json_log += "\"local_search\":"
    + std::to_string(computing_times.local_search) + "}";
  if(_cl_args.loader == 0 and _cl_args.geometry){
    // Log route information timing when using OSRM.
    json_log += ",\"detailed_geometry\":" + std::to_string(route_summary_duration);
  }
  json_log += "},";

  // Solution found.
  auto locations = instance.get_locations();
  auto m = instance.get_matrix();
  std::string tour_str = "\"route\":[";
  for(auto step = tour.cbegin(); step != tour.cend(); ++step){
    tour_str += "[" + std::to_string(locations[*step].first)
      + "," + std::to_string(locations[*step].second) + "],";
  }
  tour_str.pop_back();          // Remove trailing comma.
  tour_str += "],";

  json_log += tour_str;

  json_log += "\"solution_cost\":" + std::to_string(instance.cost(tour)) + ",";

  if(_cl_args.loader == 0 and _cl_args.geometry){
    // Add route informations summary when using OSRM.
    json_log += route_summary;
  }
  else{
    json_log.pop_back();        // Remove trailing comma.
  }
  
  json_log += "}";
  
  return json_log;
}

void logger::tour_to_output(const tsp& instance,
                            const std::list<index_t>& tour,
                            const timing_t& computing_times) const{
  std::string log_str = this->tour_to_string(instance, tour, computing_times);
  
  if(_cl_args.output_file.empty()){
    // Log to standard output.
    std::cout << log_str << std::endl;
  }
  else{
    // Log to file given as command-line option.
    std::ofstream out_stream (_cl_args.output_file, std::ofstream::out);
    out_stream << log_str;
    out_stream.close();
  }
}
