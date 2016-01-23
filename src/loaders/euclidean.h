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
#include "./problem_io.h"
#include "../structures/matrix.h"
#include "../utils/exceptions.h"

class euclidean : public problem_io<distance_t>{

private:
  std::vector<std::pair<double, double>> _locations;

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
    return virtual_euclidian_matrix<distance_t>(_locations);
  }

  virtual void get_route(const std::list<index_t>& tour,
                         rapidjson::Value& value,
                         rapidjson::Document::AllocatorType& allocator) const override{
    rapidjson::Value route_array(rapidjson::kArrayType);
  }

  virtual void get_tour(const std::list<index_t>& tour,
                        rapidjson::Value& value,
                        rapidjson::Document::AllocatorType& allocator) const override{
  }

  virtual void get_route_infos(const std::list<index_t>& tour,
                               rapidjson::Document& output) const override{
  }
};

#endif
