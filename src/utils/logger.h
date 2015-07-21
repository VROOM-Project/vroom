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

#ifndef LOGGER_H
#define LOGGER_H
#include <iostream>
#include <fstream>
#include <string>
#include <list>
#include <chrono>
#include "../structures/tsp.h"

class logger{
protected:
  std::string _file_name;
  
public:
  logger(std::string file_name);
  
  std::string tour_to_string(const tsp& instance,
                             const std::list<unsigned>& tour,
                             double computing_time) const;

  void tour_to_file(const tsp& instance,
                    const std::list<unsigned>& tour,
                    double computing_time) const;

  void places_to_file(const tsp& instance) const;
};

#endif
