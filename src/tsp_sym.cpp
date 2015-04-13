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

tsp_sym::tsp_sym(std::vector<std::vector<unsigned>> matrix)
  : tsp(matrix)
{
  unsigned size = matrix.size();
  // Checking for a symmetric matrix
  bool matrix_ok = true;
  for(unsigned i = 0; i < size; ++i){
    matrix_ok &= (matrix[i][i] == 0);
    for(unsigned j = i + 1; j < size; ++j){
      matrix_ok &= (matrix[i][j] == matrix[j][i]);
    }
  }
  if(!matrix_ok){
    std::cout << "Error in input matrix, symmetric matrix required!\n";
    exit(1);
  }
}
