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

#ifndef EUCLIDEAN_H
#define EUCLIDEAN_H
#include <vector>
#include <regex>
#include "./problem_io.h"
#include "../structures/matrix.h"
#include "../utils/exceptions.h"

class euclidean : public problem_io<distance_t>{

private:
  std::vector<std::pair<double, double>> _locations;

  void add_location(const std::string location){
    // Regex check for valid location.
    std::regex valid_loc ("loc=-?[0-9]+\\.?[0-9]*,-?[0-9]+\\.?[0-9]*[[:space:]]*");
    if(!std::regex_match(location, valid_loc)){
      throw custom_exception("invalid syntax for location "
                             + std::to_string(_locations.size() + 1)
                             + ", see vroom -h for usage display."
                             );
    }

    // Parsing the location is now safe.
    std::size_t separator_rank = location.find(",");
    std::string lat = location.substr(4, separator_rank);
    std::string lon = location.substr(separator_rank + 1, location.length() -1);
    _locations.emplace_back(std::stod(lat, nullptr),
                            std::stod(lon, nullptr));
  }
  
public:
  euclidean(std::string loc_input){
    // Parsing input in locations.
    std::size_t start = 0;
    std::size_t end = loc_input.find("&", start);
    while(end != std::string::npos){
      this->add_location(loc_input.substr(start, end - start));
      start = end + 1;
      end = loc_input.find("&", start);
    }
    // Adding last element, after last "&".
    end = loc_input.length();
    this->add_location(loc_input.substr(start, end - start));

    if(_locations.size() <= 1){
      throw custom_exception("at least two locations required!");
    }
  }

  virtual matrix<distance_t> get_matrix() const override{
    return matrix<distance_t>(_locations);
  }

  virtual void get_route(const std::list<index_t>& tour,
                         rapidjson::Value& value,
                         rapidjson::Document::AllocatorType& allocator) const override{
    rapidjson::Value route_array(rapidjson::kArrayType);
    for(auto const& step: tour){
      route_array
        .PushBack(rapidjson::Value(rapidjson::kArrayType)
                  .PushBack(_locations[step].first, allocator)
                  .PushBack(_locations[step].second, allocator),
                  allocator);
    }
    value.Swap(route_array);
  }

  virtual void get_tour(const std::list<index_t>& tour,
                        rapidjson::Value& value,
                        rapidjson::Document::AllocatorType& allocator) const override{
    rapidjson::Value tour_array(rapidjson::kArrayType);
    for(auto const& step: tour){
      // Using input index to describe locations.
      tour_array.PushBack(step, allocator);
    }
    value.Swap(tour_array);
  }

  virtual void get_route_infos(const std::list<index_t>& tour,
                               rapidjson::Document& output) const override{
  }
};

#endif
