/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./routed_wrapper.h"

routed_wrapper::routed_wrapper(const std::string& address,
                               const std::string& port,
                               const std::string& osrm_profile)
  : osrm_wrapper(osrm_profile), _address(address), _port(port) {
}

std::string
routed_wrapper::build_query(const std::vector<location_t>& locations,
                            std::string service,
                            std::string extra_args = "") const {
  // Building query for osrm-routed
  std::string query = "GET /" + service;

  query += "/v1/" + _osrm_profile + "/";

  // Adding locations.
  for (auto const& location : locations) {
    query += std::to_string(location.lon()) + "," +
             std::to_string(location.lat()) + ";";
  }
  query.pop_back(); // Remove trailing ';'.

  if (!extra_args.empty()) {
    query += "?" + extra_args;
  }

  query += " HTTP/1.1\r\n";
  query += "Host: " + _address + "\r\n";
  query += "Accept: */*\r\n";
  query += "Connection: close\r\n\r\n";

  return query;
}

std::string routed_wrapper::send_then_receive(std::string query) const {
  std::string response;

  try {
    boost::asio::io_service io_service;

    tcp::resolver r(io_service);
    tcp::resolver::query q(_address, _port);

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
    throw custom_exception("Failure while connecting to the OSRM server.");
  }
  return response;
}

matrix<cost_t>
routed_wrapper::get_matrix(const std::vector<location_t>& locs) const {
  std::string query = this->build_query(locs, "table");

  std::string response = this->send_then_receive(query);

  // Removing headers.
  std::string json_content = response.substr(response.find("{"));

  // Expected matrix size.
  std::size_t m_size = locs.size();

  // Checking everything is fine in the response.
  rapidjson::Document infos;
  assert(!infos.Parse(json_content.c_str()).HasParseError());
  assert(infos.HasMember("code"));
  if (infos["code"] != "Ok") {
    throw custom_exception("OSRM table: " +
                           std::string(infos["message"].GetString()));
  }
  assert(infos["durations"].Size() == m_size);

  // Build matrix while checking for unfound routes to avoid
  // unexpected behavior (OSRM raises 'null').
  matrix<cost_t> m(m_size);

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

void routed_wrapper::add_route_info(route_t& rte) const {
  // Ordering locations for the given steps.
  std::vector<location_t> ordered_locations;
  for (const auto& step : rte.steps) {
    ordered_locations.push_back(step.location);
  }

  std::string extra_args =
    "alternatives=false&steps=false&overview=full&continue_straight=false";

  std::string query = this->build_query(ordered_locations, "route", extra_args);
  std::string response = this->send_then_receive(query);

  // Removing headers
  std::string json_content = response.substr(response.find("{"));

  // Checking everything is fine in the response.
  rapidjson::Document infos;

  assert(!infos.Parse(json_content.c_str()).HasParseError());
  assert(infos.HasMember("code"));
  if (infos["code"] != "Ok") {
    throw custom_exception("OSRM route: " +
                           std::string(infos["message"].GetString()));
  }

  // Parse total time/distance and route geometry.
  rte.duration = round_cost(infos["routes"][0]["duration"].GetDouble());
  rte.distance = round_cost(infos["routes"][0]["distance"].GetDouble());
  rte.geometry = std::move(infos["routes"][0]["geometry"].GetString());

  rte.service = std::accumulate(rte.steps.begin(),
                                rte.steps.end(),
                                0,
                                [](const duration_t& sum, const auto& step) {
                                  return sum + step.service;
                                });

  auto nb_legs = infos["routes"][0]["legs"].Size();
  assert(nb_legs == rte.steps.size() - 1);
  double current_distance = 0;
  double current_duration = 0;

  rte.steps[0].distance = round_cost(current_distance);
  rte.steps[0].arrival = round_cost(current_duration);

  for (rapidjson::SizeType i = 0; i < nb_legs; ++i) {
    current_distance += infos["routes"][0]["legs"][i]["distance"].GetDouble();
    current_duration += infos["routes"][0]["legs"][i]["duration"].GetDouble();

    rte.steps[i + 1].distance = round_cost(current_distance);
    rte.steps[i + 1].arrival = round_cost(current_duration);
  }
}
