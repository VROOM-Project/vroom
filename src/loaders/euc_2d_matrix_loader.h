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

#ifndef EUC_2D_MATRIX_LOADER_H
#define EUC_2D_MATRIX_LOADER_H
#include <vector>
#include <cmath>
#include "./matrix_loader.h"
#include "../structures/matrix.h"

class euc_2d_matrix_loader : public matrix_loader<unsigned, double>{

public:
  virtual matrix<unsigned> load_matrix(const std::vector<std::pair<double, double>>& places){
    unsigned n = places.size();
    std::vector<unsigned> blank_line (n, 0);
    std::vector<std::vector<unsigned>> matrix_as_vector (n, blank_line);

    for(unsigned i = 0; i < n; ++i){
      matrix_as_vector[i][i] = 0;
      for(unsigned j = i + 1; j < n; ++j){
        double xd = places[j].first - places[i].first;
        double yd = places[j].second - places[i].second;
        unsigned dij = nint(std::sqrt(xd * xd + yd * yd));
        matrix_as_vector[i][j] = dij;
        matrix_as_vector[j][i] = dij;
      }
    }
    matrix<unsigned> m (matrix_as_vector);
    return m;
  }
};

#endif
