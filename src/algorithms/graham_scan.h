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

#ifndef GRAHAM_SCAN_H
#define GRAHAM_SCAN_H
#include <map>
#include <vector>
#include <algorithm>            // sort

template <class T>
T orientation(const std::pair<T, T>& p1,
              const std::pair<T, T>& p2,
              const std::pair<T, T>& p3){
  return (p2.first - p1.first) * (p3.second - p1.second)
    - (p2.second - p1.second) * (p3.first - p1.first);
}

template <class T>
std::list<index_t> convex_hull(const std::vector<std::pair<T, T>>& locations){
  // Computes the points on the convex hull of "locations". Return a
  // counter-clockwise tour of the convex hull boundary: a list of the
  // index of the corresponding points in locations.

  unsigned nb_pts = locations.size();

  // Used to get back index of a given location in locations.
  std::map<std::pair<T, T>, index_t> location_indices;

  // Determine lowest location as a first point on the convex hull.
  std::pair<T, T> lowest_location = locations[0];
  location_indices.emplace(locations[0], 0);
  auto location = locations.begin();
  index_t index = 1;
  for(++location; location != locations.end(); ++location){
    location_indices.emplace(*location, index);
    ++index;
    if((location->second < lowest_location.second)
       or ((location->second == lowest_location.second)
           and (location->first < lowest_location.first))){
      lowest_location = *location;
    }
  }
  
  // Using a sorted copy of locations.
  std::vector<std::pair<T, T>> sorted_locations (locations);

  struct orientation_order{
    std::pair<T, T> _ref;

    orientation_order(std::pair<T, T> ref):
      _ref(ref) {}

    bool operator()(const std::pair<T, T>& lhs,
                    const std::pair<T, T>& rhs){
      bool is_inferior;
      T lh_y_diff = lhs.second - _ref.second;
      T rh_y_diff = rhs.second - _ref.second;
      T lh_x_diff = lhs.first - _ref.first;
      T rh_x_diff = rhs.first - _ref.first;

      is_inferior = (rhs != _ref);      // _ref is not greater than anyone.

      T orient = orientation(_ref, lhs, rhs);
      is_inferior &= (orient >= 0);
      
      if(orient == 0){
        is_inferior &= ((lh_x_diff * lh_x_diff) + (lh_y_diff * lh_y_diff)
                        < (rh_x_diff * rh_x_diff) + (rh_y_diff * rh_y_diff));
      }
      return is_inferior;

    }
  } comp (lowest_location);

  std::sort(sorted_locations.begin(), sorted_locations.end(), comp);

  size_t M = 1;
  for(size_t i = 2; i < nb_pts; ++i){
    while(orientation(sorted_locations[M - 1],
                      sorted_locations[M],
                      sorted_locations[i]) < 0){
      // Shouldn' happen when M is 1.
      --M;
    }
    ++M;
    if(M != i){
      // Swap elements if necessary.
      std::pair<T, T> temp = sorted_locations[i];
      sorted_locations[i] = sorted_locations[M];
      sorted_locations[M] = temp;
    }
  }

  // Convex hull tour.
  std::list<index_t> convex_hull;
  for(unsigned i = 0; i < M; ++i){
    convex_hull.push_back(location_indices.at(sorted_locations[i]));
  }

  return convex_hull;
}

#endif
