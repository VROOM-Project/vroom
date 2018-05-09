/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "utils/exceptions.h"

custom_exception::custom_exception(const std::string message)
  : _message(message) {
}

const std::string custom_exception::get_message() const {
  return _message;
}
