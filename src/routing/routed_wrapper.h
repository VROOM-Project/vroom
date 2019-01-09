#ifndef ROUTED_WRAPPER_H
#define ROUTED_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <unordered_map>

#include "routing/osrm_wrapper.h"

namespace vroom {
namespace routing {

using Servers = std::unordered_map<std::string, Server>;

class RoutedWrapper : public OSRMWrapper {

private:
  Servers _servers;
  std::string _host;
  std::string _port;

  void update_server();

  std::string build_query(const std::vector<Location>& locations,
                          std::string service,
                          std::string extra_args) const;

  std::string send_then_receive(std::string query) const;

public:
  RoutedWrapper(const Servers& servers);

  virtual Matrix<Cost>
  get_matrix(const std::vector<Location>& locs) const override;

  virtual void add_route_info(Route& route) const override;

  virtual void set_profile(const std::string& profile) override;
};

} // namespace routing
} // namespace vroom

#endif
