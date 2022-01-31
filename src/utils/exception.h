#ifndef EXCEPTION_H
#define EXCEPTION_H

/*

This file is part of VROOM.

Copyright (c) 2015-2022, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

#include "structures/typedefs.h"

namespace vroom {

class Exception : public std::exception {
public:
  const std::string message;
  unsigned int error_code;

  Exception(const std::string& message, unsigned int error_code);

  const char* what() const noexcept override {
    return message.c_str();
  };
};

class InternalException : public Exception {
public:
  InternalException(const std::string& message);
};

class InputException : public Exception {
public:
  InputException(const std::string& message);
};

class RoutingException : public Exception {
public:
  RoutingException(const std::string& message);
};

} // namespace vroom

#endif
