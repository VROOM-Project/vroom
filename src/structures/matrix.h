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

#ifndef MATRIX_H
#define MATRIX_H
#include <iostream>
#include <vector>

class matrix{

private:
  std::size_t _size;
  std::vector<std::vector<unsigned>> _inner_rep;
  
public:
  matrix(std::vector<std::vector<unsigned>> matrix_as_vector);

  std::size_t size() const;

  void print() const;

  matrix get_sub_matrix(const std::vector<unsigned>& indices) const;

  unsigned operator()(unsigned i, unsigned j) const;
};

#endif
