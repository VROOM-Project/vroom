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

logger::logger(std::string file_name):
  _file_name(file_name) {}

std::string logger::tour_to_string(const tsp& instance,
                                   const std::list<index_t>& tour,
                                   const timing_t& computing_times) const{
  auto places = instance.get_places();
  auto m = instance.get_matrix();
  std::string places_str = "\"places_tour\":[";
  std::string lengths_str = "\"lengths\":[";
  std::string indices_str = "\"indices\":[";
  auto step = tour.cbegin();
  while(step != tour.cend()){
    indices_str += std::to_string(*step) + ",";
    places_str += "{\"lat\":" + std::to_string(places[*step].first)
      + ",\"lon\":" + std::to_string(places[*step].second) + "},";
    auto current_step = step;
    ++step;
    if(step != tour.cend()){
      lengths_str += std::to_string(m(*current_step, *step)) + ",";
    }
  }
  indices_str.pop_back();
  indices_str += "],";
  places_str.pop_back();
  places_str += "],";

  lengths_str += std::to_string(m(tour.back(), tour.front())) + "],";

  std::string json_log = "{" + indices_str + places_str + lengths_str;

  json_log += "\"total_length\":" + std::to_string(instance.cost(tour)) + ",";
  json_log += "\"computing_times\":{";
  json_log += "\"matrix_loading\":"
    + std::to_string(computing_times.matrix_loading) + ",";
  json_log += "\"heuristic\":"
    + std::to_string(computing_times.heuristic) + ",";
  json_log += "\"local_search\":" + std::to_string(computing_times.local_search);
  json_log += "}}";

  return json_log;
}

void logger::tour_to_file(const tsp& instance,
                          const std::list<index_t>& tour,
                          const timing_t& computing_times) const{
  auto timestamp
    = std::chrono::system_clock::now().time_since_epoch().count();
  std::ofstream out_stream (std::to_string(timestamp) + "_" + _file_name,
                            std::ofstream::out);
  out_stream << this->tour_to_string(instance, tour, computing_times);
  out_stream.close();

  // std::cout << instance.get_route_summary(tour) << std::endl;
}
