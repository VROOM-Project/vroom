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

#include "tsp_sym.h"

tsp_sym::tsp_sym(std::string places){
  // Poorly filtered for now.
  std::size_t start = 4;
  std::size_t end = places.find("&", start);
  while(end != std::string::npos){
    std::size_t separator_rank = places.find(",", start);
    std::string lat = places.substr(start,
                                    separator_rank - start);
    std::string lon = places.substr(separator_rank + 1,
                                          end - separator_rank - 1);
    _places.emplace_back(std::stod(lat, nullptr),
                         std::stod(lon, nullptr));
       
    start = end + 5;
    end = places.find("&", start);
  }
  // Adding last element, after last "&".
  end = places.length();
  std::size_t separator_rank = places.find(",", start);
  std::string lat = places.substr(start,
                                  separator_rank - start);
  std::string lon = places.substr(separator_rank + 1,
                                  end - separator_rank - 1);
  _places.emplace_back(std::stod(lat, nullptr),
                       std::stod(lon, nullptr));

  // Computing matrix.
  euc_2d_matrix_loader loader;
  _matrix = loader.load_matrix(_places);
  undirected_graph<unsigned> graph (_matrix);
  _graph = graph;
}

tsp_sym::tsp_sym(matrix<unsigned> m)
  : tsp(m), _graph(m) {}

undirected_graph<unsigned> tsp_sym::get_graph() const{
  return _graph;
}

std::string tsp_sym::log(const std::list<unsigned>& tour) const{
  std::string places = "\"places_tour\":[";
  std::string lengths = "\"lengths\":[";
  std::string indices = "\"indices\":[";
  auto step = tour.cbegin();
  while(step != tour.cend()){
    indices += std::to_string(*step) + ",";
    places += "{\"lat\":" + std::to_string(_places[*step].first)
      + ",\"lon\":" + std::to_string(_places[*step].second) + "},";
    auto current_step = step;
    ++step;
    if(step != tour.cend()){
      lengths += std::to_string(_matrix(*current_step, *step)) + ",";
    }
  }
  indices.pop_back();
  indices += "],";
  places.pop_back();
  places += "],";

  lengths += std::to_string(_matrix(tour.back(), tour.front())) + "],";

  std::string json_log = "{" + indices + places + lengths;

  json_log += "\"total_length\":" + std::to_string(this->cost(tour));
  json_log += "}";

  return json_log;
}
