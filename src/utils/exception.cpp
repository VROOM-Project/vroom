/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "utils/exception.h"

namespace vroom {

Exception::Exception(ERROR error,
                     const std::string& message,
                     unsigned int error_code)
  : error(error), message(message), error_code(error_code) {
}

InternalException::InternalException(const std::string& message)
  : Exception(ERROR::INTERNAL, message, 1) {
}

InputException::InputException(const std::string& message)
  : Exception(ERROR::INPUT, message, 2) {
}

RoutingException::RoutingException(const std::string& message)
  : Exception(ERROR::ROUTING, message, 3) {
}

} // namespace vroom
