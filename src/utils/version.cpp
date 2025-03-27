/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "utils/version.h"

namespace vroom {

std::string get_version() {
  std::string version = std::format("{}.{}.{}", MAJOR, MINOR, PATCH);
  if (DEV) {
    version += "-dev";
  } else {
    if (RC > 0) {
      version += std::format("-rc.{}", RC);
    }
  }
  return version;
}

} // namespace vroom
