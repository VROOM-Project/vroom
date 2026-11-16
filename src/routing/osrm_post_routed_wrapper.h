#ifndef OSRM_POST_ROUTED_WRAPPER_H
#define OSRM_POST_ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_routed_wrapper.h"

namespace vroom::routing {

class OsrmPostRoutedWrapper : public OsrmRoutedWrapper {
private:
  std::string build_query(const std::vector<Location>& locations,
                          const std::string& service) const override;

  std::string build_request_body(const std::vector<Location>& locations,
                                 const std::string& service) const;

public:
  OsrmPostRoutedWrapper(const std::string& profile, const Server& server);
};

} // namespace vroom::routing

#endif
