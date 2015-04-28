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

#include "edge.h"

edge::edge(unsigned first_vertex,
           unsigned second_vertex,
           unsigned weight):
  _first_vertex(std::min(first_vertex, second_vertex)),
  _second_vertex(std::max(first_vertex, second_vertex)),
  _weight(weight) {}

unsigned edge::get_first_vertex() const{
  return _first_vertex;
}

unsigned edge::get_second_vertex() const{
  return _second_vertex;
}
  
bool edge::operator<(const edge& rhs) const{
  return (this->_first_vertex < rhs._first_vertex)
    or ((this->_first_vertex == rhs._first_vertex)
        and (this->_second_vertex < rhs._second_vertex));
}

bool edge::operator==(const edge& rhs) const{
  return (this->_first_vertex == rhs._first_vertex)
    and (this->_second_vertex == rhs._second_vertex);
}

unsigned edge::get_weight() const{
  return _weight;
}

void edge::log() const{
  std::cout << _first_vertex
            << "<-(" << _weight << ")->"
            << _second_vertex;
}
