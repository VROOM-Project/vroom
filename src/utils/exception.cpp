/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "utils/exception.h"

namespace vroom {

Exception::Exception(std::string message, unsigned int error_code)
  : message(std::move(message)), error_code(error_code) {
}

InternalException::InternalException(const std::string& message)
  : Exception(message, 1) {
}

InputException::InputException(const std::string& message)
  : Exception(message, 2) {
}

RoutingException::RoutingException(const std::string& message)
  : Exception(message, 3) {
}

} // namespace vroom
