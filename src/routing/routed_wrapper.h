#ifndef ROUTED_WRAPPER_H
#define ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "routing/osrm_wrapper.h"

namespace vroom {

class RoutedWrapper : public OSRMWrapper {

private:
  std::string _address;
  std::string _port;

  std::string build_query(const std::vector<Location>& locations,
                          std::string service,
                          std::string extra_args) const;

  std::string send_then_receive(std::string query) const;

public:
  RoutedWrapper(const std::string& address,
                const std::string& port,
                const std::string& osrm_profile);

  virtual Matrix<Cost>
  get_matrix(const std::vector<Location>& locs) const override;

  virtual void add_route_info(Route& route) const override;
};

} // namespace vroom

#endif
