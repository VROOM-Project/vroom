#ifndef OSRM_ROUTED_WRAPPER_H
#define OSRM_ROUTED_WRAPPER_H

#include "http_wrapper.h"

namespace vroom::routing {
class OsrmRoutedWrapper : public HttpWrapper {

public:
  OsrmRoutedWrapper(const std::string& profile,
                    const Server& server,
                    std::string routing_arguments);

protected:
  virtual std::string build_query(const std::vector<Location>& locations,
                                  const std::string& service) const = 0;

  void check_response(const rapidjson::Document& json_result,
                      const std::vector<Location>& locs,
                      const std::string& service) const override;

  const rapidjson::Value&
  get_legs(const rapidjson::Value& result) const override;
};
} // namespace vroom::routing

#endif