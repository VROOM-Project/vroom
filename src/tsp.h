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
#include "./structures/matrix.h"
#include "./structures/matrix_loader.h"
#include "./structures/euc_2d_matrix_loader.h"

class tsp{
protected:
  std::vector<std::pair<double, double>> _places;
  matrix<unsigned> _matrix;
  
public:
  tsp();

  tsp(std::string places);
  
  tsp(matrix<unsigned> m);

  const matrix<unsigned>& get_matrix() const;

  const std::vector<std::pair<double, double>>& get_places() const;

  const matrix<unsigned> get_symmetrized_matrix() const;

  std::size_t size();

  double cost(const std::list<unsigned>& tour) const;

  std::string log(const std::list<unsigned>& tour) const;

  void log_to_file(const std::list<unsigned>& tour,
                   std::string file_name) const;

  void log_places_to_file(std::string file_name) const;
};

#endif
