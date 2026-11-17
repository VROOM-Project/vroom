#ifndef OSRM_GET_WRAPPER_H
#define OSRM_GET_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_routed_wrapper.h"

namespace vroom::routing {

class OsrmGetWrapper : public OsrmRoutedWrapper {
private:
  const std::string _routing_args{
    "alternatives=false&steps=false&overview="
    "full&continue_straight=false"};

  std::string build_query(const std::vector<Location>& locations,
                          const std::string& service) const override;

  std::string build_query_body(const std::vector<Location>& locations,
                               const std::string& service) const;

public:
  OsrmGetWrapper(const std::string& profile, const Server& server);
};

} // namespace vroom::routing

#endif
