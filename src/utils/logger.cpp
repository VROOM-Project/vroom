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
  // Execution informations.
  std::string json_log = "{\"computing_times\":{";
  json_log += "\"matrix_loading\":"
    + std::to_string(computing_times.matrix_loading) + ",";
  json_log += "\"heuristic\":"
    + std::to_string(computing_times.heuristic) + ",";
  json_log += "\"local_search\":" + std::to_string(computing_times.local_search);
  json_log += "},";

  // Solution found.
  auto places = instance.get_places();
  auto m = instance.get_matrix();
  std::string tour_str = "\"tour\":[";
  for(auto step = tour.cbegin(); step != tour.cend(); ++step){
    tour_str += "[" + std::to_string(places[*step].first)
      + "," + std::to_string(places[*step].second) + "],";
  }
  tour_str.pop_back();          // Remove trailing comma.
  tour_str += "],";

  json_log += tour_str;

  json_log += "\"total_length\":" + std::to_string(instance.cost(tour)) + ",";

  if(_cl_args.loader == 0){
    // Add route informations summary when using OSRM.
    json_log += instance.get_route_summary(tour);
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
