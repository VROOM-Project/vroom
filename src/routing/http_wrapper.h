#ifndef HTTP_WRAPPER_H
#define HTTP_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"
#include "structures/vroom/location.h"

namespace vroom {
namespace routing {

class HttpWrapper {

protected:
  const Server _server;

  HttpWrapper(const Server& server);

  std::string send_then_receive(std::string query) const;

  virtual std::string build_query(const std::vector<Location>& locations,
                                  std::string service,
                                  std::string extra_args) const = 0;
};

} // namespace routing
} // namespace vroom

#endif
