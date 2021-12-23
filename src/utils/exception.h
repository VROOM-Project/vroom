#ifndef EXCEPTION_H
#define EXCEPTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2021, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/typedefs.h"

namespace vroom {

class Exception : public std::exception {
public:
  const ERROR error;
  const std::string message;

  Exception(ERROR error, const std::string& message);

  const char* what() const noexcept override { return message.c_str(); }
};

} // namespace vroom

#endif
