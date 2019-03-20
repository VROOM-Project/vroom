/*

This file is part of VROOM.

Copyright (c) 2015-2019, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "../include/rapidjson/document.h"
#include "../include/rapidjson/error/en.h"
#include <boost/asio.hpp>

#include "routing/orshttp_wrapper.h"
#include "utils/exception.h"

using boost::asio::ip::tcp;

namespace vroom {
namespace routing {

OrsHttpWrapper::OrsHttpWrapper(const std::string& profile, const Server& server)
  : OSRMWrapper(profile), _server(server) {
}

std::string OrsHttpWrapper::build_query(const std::vector<Location>& locations,
                                        std::string service,
                                        std::string extra_args = "") const {
  // Adding locations.
  std::string body = "{\"";
  if (service == "directions") {
    body += "coordinates";
  } else {
    body += "locations";
  }
  body += "\":[";
  for (auto const& location : locations) {
    body += "[" + std::to_string(location.lon()) + "," +
            std::to_string(location.lat()) + "],";
  }
  body.pop_back(); // Remove trailing ','.
  body += "]";
  if (!extra_args.empty()) {
    body += "," + extra_args;
  }
  body += "}";

  // Building query for ORS
  std::string query = "POST /ors/v2/" + service + "/" + _profile;

  query += " HTTP/1.0\r\n";
  query += "Accept: */*\r\n";
  query += "Content-Type: application/json\r\n";
  query += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  query += "Host: " + _server.host + ":" + _server.port + "\r\n";
  query += "Connection: close\r\n";
  query += "\r\n" + body;

  return query;
}

std::string OrsHttpWrapper::send_then_receive(std::string query) const {
  std::string response;

  try {
    boost::asio::io_service io_service;

    tcp::resolver r(io_service);

    tcp::resolver::query q(_server.host, _server.port);

    tcp::socket s(io_service);
    boost::asio::connect(s, r.resolve(q));

    boost::asio::write(s, boost::asio::buffer(query));

    char buf[512];
    boost::system::error_code error;
    for (;;) {
      std::size_t len = s.read_some(boost::asio::buffer(buf), error);
      response.append(buf, len);
      if (error == boost::asio::error::eof) {
        // Connection closed cleanly.
        break;
      } else {
        if (error) {
          throw boost::system::system_error(error);
        }
      }
    }
  } catch (boost::system::system_error& e) {
    throw Exception(ERROR::ROUTING,
                    "Failed to connect to ORS at " + _server.host + ":" +
                      _server.port);
  }
  return response;
}

Matrix<Cost>
OrsHttpWrapper::get_matrix(const std::vector<Location>& locs) const {
  std::string query = this->build_query(locs, "matrix");
  std::string response = this->send_then_receive(query);

  // Removing headers.
  std::size_t json_start = response.find("{");
  std::size_t json_length = response.rfind("}") - json_start + 1;
  std::string json_content = response.substr(json_start, json_length);

  // Expected matrix size.
  std::size_t m_size = locs.size();

  // Checking everything is fine in the response.
  rapidjson::Document infos;
#ifdef NDEBUG
  infos.Parse(json_content.c_str());
#else
  assert(!infos.Parse(json_content.c_str()).HasParseError());
#endif
  if (infos.HasMember("error")) {
    throw Exception(ERROR::ROUTING,
                    "ORS matrix:" +
                      std::string(infos["error"]["message"].GetString()));
  }

  assert(infos["durations"].Size() == m_size);

  // Build matrix while checking for unfound routes to avoid
  // unexpected behavior (ORS raises 'null').
  Matrix<Cost> m(m_size);

  std::vector<unsigned> nb_unfound_from_loc(m_size, 0);
  std::vector<unsigned> nb_unfound_to_loc(m_size, 0);

  for (rapidjson::SizeType i = 0; i < infos["durations"].Size(); ++i) {
    const auto& line = infos["durations"][i];
    assert(line.Size() == m_size);
    for (rapidjson::SizeType j = 0; j < line.Size(); ++j) {
      if (line[j].IsNull()) {
        // No route found between i and j. Just storing info as we
        // don't know yet which location is responsible between i
        // and j.
        ++nb_unfound_from_loc[i];
        ++nb_unfound_to_loc[j];
      } else {
        auto cost = round_cost(line[j].GetDouble());
        m[i][j] = cost;
      }
    }
  }

  check_unfound(locs, nb_unfound_from_loc, nb_unfound_to_loc);

  return m;
}

void OrsHttpWrapper::add_route_info(Route& route) const {
  // Ordering locations for the given steps.
  std::vector<Location> ordered_locations;
  for (const auto& step : route.steps) {
    ordered_locations.push_back(step.location);
  }

  std::string extra_args =
    "\"geometry_simplify\":\"false\",\"continue_straight\":\"false\"";

  std::string query =
    this->build_query(ordered_locations, "directions", extra_args);
  std::string response = this->send_then_receive(query);

  // Removing headers
  std::size_t json_start = response.find("{");
  std::size_t json_length = response.rfind("}") - json_start + 1;
  std::string json_content = response.substr(json_start, json_length);

  // Checking everything is fine in the response.
  rapidjson::Document infos;
#ifdef NDEBUG
  infos.Parse(json_content.c_str());
#else
  assert(!infos.Parse(json_content.c_str()).HasParseError());
#endif
  if (infos.HasMember("error")) {
    throw Exception(ERROR::ROUTING,
                    "ORS routes: " +
                      std::string(infos["error"]["message"].GetString()));
  }

  // Total distance and route geometry.
  route.distance =
    round_cost(infos["routes"][0]["summary"]["distance"].GetDouble());
  route.geometry = std::move(infos["routes"][0]["geometry"].GetString());

  auto nb_legs = infos["routes"][0]["segments"].Size();
  assert(nb_legs == route.steps.size() - 1);

  // Accumulated travel distance stored for each step.
  double current_distance = 0;

  route.steps[0].distance = 0;

  for (rapidjson::SizeType i = 0; i < nb_legs; ++i) {
    // Update distance for step after current route leg.
    current_distance +=
      infos["routes"][0]["segments"][i]["distance"].GetDouble();
    route.steps[i + 1].distance = round_cost(current_distance);
  }
}

} // namespace routing
} // namespace vroom
