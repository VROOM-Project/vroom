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

#ifndef TSP_H
#define TSP_H
#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <regex>
#include "./typedefs.h"
#include "./matrix.h"
#include "../loaders/matrix_loader.h"
#include "../loaders/euc_2d_matrix_loader.h"
#include "../loaders/osrm_wrapper.h"

class tsp{
protected:
  std::vector<std::pair<double, double>> _locations;
  matrix<distance_t> _matrix;
  
private:
  void add_location(const std::string location);

public:
  tsp(const cl_args_t& cl_args);
  
  tsp(matrix<distance_t> m);

  const matrix<distance_t>& get_matrix() const;

  const std::vector<std::pair<double, double>>& get_locations() const;

  const matrix<distance_t> get_symmetrized_matrix() const;

  std::size_t size();

  distance_t cost(const std::list<index_t>& tour) const;

  std::string get_route_summary(const std::list<index_t>& tour) const;
};

#endif
