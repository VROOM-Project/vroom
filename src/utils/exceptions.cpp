/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015-2016, Julien Coupey

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

#include "exceptions.h"

custom_exception::custom_exception(const std::string message):
  _message(message){}

const std::string custom_exception::get_message() const{
  return _message;
}
