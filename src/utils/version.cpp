/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "utils/version.h"

std::string get_version() {
  std::string version = std::to_string(MAJOR) + "." + std::to_string(MINOR) +
                        "." + std::to_string(PATCH);
  if (DEV) {
    version += "-dev";
  } else {
    if (RC) {
      version += "-rc." + std::to_string(RC);
    }
  }
  return version;
}
