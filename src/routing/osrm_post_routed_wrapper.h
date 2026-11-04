#ifndef OSRM_POST_ROUTED_WRAPPER_H
#define OSRM_POST_ROUTED_WRAPPER_H

#include "routing/http_wrapper.h"
#include <map>

namespace vroom::routing {

class OsrmPostRoutedWrapper : public HttpWrapper {
private:
  std::string build_query(const std::vector<Location>& locations,
                          const std::string& service) const override;

  void check_response(const rapidjson::Document& json_result,
                      const std::vector<Location>& locs,
                      const std::string& service) const override;

  const rapidjson::Value&
  get_legs(const rapidjson::Value& result) const override;

  std::string build_request_body(const std::vector<Location>& locations,
                                 const std::string& service) const;

  std::map<std::string, rapidjson::Value> get_routing_args_as_map() const;

  std::vector<std::string> split(const std::string& str, char delimiter) const;

  std::string json_to_string(const rapidjson::Document& docs) const;

public:
  OsrmPostRoutedWrapper(const std::string& profile, const Server& server);
};

} // namespace vroom::routing

#endif