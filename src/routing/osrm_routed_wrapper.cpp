#include "osrm_routed_wrapper.h"

namespace vroom::routing {

OsrmRoutedWrapper::OsrmRoutedWrapper(const std::string& profile,
                                     const Server& server,
                                     std::string routing_arguments)
  : HttpWrapper(profile,
                server,
                "table",
                "durations",
                "distances",
                "route",
                std::move(routing_arguments)) {
}

void OsrmRoutedWrapper::check_response(const rapidjson::Document& json_result,
                                       const std::vector<Location>& locs,
                                       const std::string& service) const {
  assert(json_result.HasMember("code"));
  const std::string code = json_result["code"].GetString();
  if (code != "Ok") {
    const std::string message = json_result["message"].GetString();
    if (const std::string snapping_error_base =
          "Could not find a matching segment for coordinate ";
        code == "NoSegment" && message.starts_with(snapping_error_base)) {
      const auto error_loc =
        std::stoul(message.substr(snapping_error_base.size(),
                                  message.size() - snapping_error_base.size()));
      const auto coordinates = std::format("[{:.6f},{:.6f}]",
                                           locs[error_loc].lon(),
                                           locs[error_loc].lat());
      throw RoutingException("Could not find route near location " +
                             coordinates);
    }

    // Other error in response.
    throw RoutingException(message);
  }
}

const rapidjson::Value&
OsrmRoutedWrapper::get_legs(const rapidjson::Value& result) const {
  assert(result.HasMember("routes") && result["routes"].IsArray() &&
         !result["routes"].Empty() && result["routes"][0].HasMember("legs") &&
         result["routes"][0]["legs"].IsArray());

  return result["routes"][0]["legs"];
}
} // namespace vroom::routing