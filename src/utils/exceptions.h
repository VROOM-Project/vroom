#ifndef EXCEPTIONS_H
#define EXCEPTIONS_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>

class Exception : public std::exception {
private:
  const std::string _message;

public:
  Exception(const std::string message);

  const std::string get_message() const;
};

#endif
