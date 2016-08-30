#ifndef ROUTED_LOADER_H
#define ROUTED_LOADER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "./osrm_loader.h"
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

class routed_loader : public osrm_loader{

private:
  std::string _address;            // OSRM server adress
  std::string _port;               // OSRM server listening port

  std::string build_query(const std::vector<Location>& locations,
                          std::string service,
                          std::string extra_args = "") const{
    // Building query for osrm-routed
    std::string query = "GET /" + service;

    if(_use_osrm_v5){
      query += "/v1/" + _osrm_profile + "/";

      // Adding locations.
      for(auto const& location: locations){
        // OSRM v5 has gone [lon,lat].
        query += std::to_string(location.lon)
          + ","
          + std::to_string(location.lat)
          + ";";
      }
      query.pop_back();         // Remove trailing ';'.

      if(!extra_args.empty()){
        query += "?" + extra_args;
      }
    }
    else{
      // Backward compat.
      query += "?";

      // Adding locations.
      for(auto const& location: locations){
        query += "loc="
          + std::to_string(location.lat)
          + ","
          + std::to_string(location.lon)
          + "&";
      }

      if(!extra_args.empty()){
        query += extra_args + "&";
      }
      query.pop_back();         // Remove trailing '&'.
    }

    query += " HTTP/1.1\r\n";
    query += "Host: " + _address + "\r\n";
    query += "Accept: */*\r\n";
    query += "Connection: close\r\n\r\n";

    return query;
  }

  std::string send_then_receive(std::string query) const{
    std::string response;

    try{
      boost::asio::io_service io_service;

      tcp::resolver r (io_service);
      tcp::resolver::query q (_address, _port);

      tcp::socket s (io_service);
      boost::asio::connect(s, r.resolve(q));

      boost::asio::write(s, boost::asio::buffer(query));

      char buf[512];
      boost::system::error_code error;
      for(;;){
        std::size_t len = s.read_some(boost::asio::buffer(buf), error);
        response.append(buf, len);
        if(error == boost::asio::error::eof){
          // Connection closed cleanly.
          break;
        }
        else{
          if(error){
            throw boost::system::system_error(error);
          }
        }
      }
    }
    catch (boost::system::system_error& e)
      {
        throw custom_exception("Failure while connecting to the OSRM server.");
      }
    return response;
  }

public:
  routed_loader(const std::string& address,
                 const std::string& port,
                 const std::string& osrm_profile,
                 const std::string& input):
    osrm_loader(osrm_profile, input),
    _address(address),
    _port(port) {}

  virtual matrix<distance_t> get_matrix() const override{
    std::string query = this->build_query(_locations, "table");

    std::string response = this->send_then_receive(query);

    if(!_use_osrm_v5){
      if(response.find("Bad Request") != std::string::npos){
        // Backward compat. Stop at "Bad Request" error from OSRM.
        throw custom_exception("OSRM v4 API error. Use -m with OSRM v5.");
      }
    }

    // Removing headers.
    std::string json_content = response.substr(response.find("{"));

    // Expected matrix size.
    std::size_t m_size = _locations.size();
    // Matrix key label depends on OSRM version.
    const char* durations = _use_osrm_v5 ? "durations": "distance_table";

    // Checking everything is fine in the response (OSRM version
    // dependant).
    rapidjson::Document infos;
    assert(!infos.Parse(json_content.c_str()).HasParseError());
    if(_use_osrm_v5){
      assert(infos.HasMember("code"));
      if(infos["code"] != "Ok"){
        throw custom_exception("OSRM table: "
                               + std::string(infos["message"].GetString()));
      }
    }
    else{
      // Backward compat.
      assert(infos.HasMember(durations));
    }
    assert(infos[durations].Size() == m_size);

    // Building matrix and checking for unfound routes to avoid
    // unexpected behavior (OSRM v5 raises 'null' while v4 raises a
    // max int value).
    matrix<distance_t> m {m_size};

    std::vector<unsigned> nb_unfound_from_loc (m_size, 0);
    std::vector<unsigned> nb_unfound_to_loc (m_size, 0);
    distance_t unfound_time
      = std::numeric_limits<int>::max();

    for(rapidjson::SizeType i = 0; i < infos[durations].Size(); ++i){
      const auto& line = infos[durations][i];
      assert(line.Size() == m_size);
      for(rapidjson::SizeType j = 0; j < line.Size(); ++j){
        if((_use_osrm_v5 and line[j].IsNull())
           or (!_use_osrm_v5 and (line[j].GetUint() == unfound_time))
           ){
          // No route found between i and j. Just storing info as we
          // don't know yet which location is responsible between i
          // and j.
          ++nb_unfound_from_loc[i];
          ++nb_unfound_to_loc[j];
        }
        else{
          m[i][j] = round_to_distance(line[j].GetDouble());
        }
      }
    }

    check_unfound(nb_unfound_from_loc, nb_unfound_to_loc);

    return m;
  }

  virtual void get_route_infos(const std::list<index_t>& steps,
                               rapidjson::Value& value,
                               rapidjson::Document::AllocatorType& allocator) const override{
    // Ordering locations for the given steps.
    std::vector<Location> ordered_locations;
    for(auto& step: steps){
      ordered_locations.push_back(_locations[step]);
    }

    // Backward compat.
    std::string route_service = _use_osrm_v5 ? "route": "viaroute";
    std::string extra_args = "alt=false&uturns=true";
    if(_use_osrm_v5){
      extra_args = "alternatives=false&steps=false&overview=full&continue_straight=false";
    }

    std::string query = this->build_query(ordered_locations,
                                          route_service,
                                          extra_args);
    std::string response = this->send_then_receive(query);

    // Removing headers
    std::string json_content = response.substr(response.find("{"));

    // Checking everything is fine in the response (OSRM version
    // dependant). Then parse total time/distance and route geometry.
    rapidjson::Document infos;

    assert(!infos.Parse(json_content.c_str()).HasParseError());
    if(_use_osrm_v5){
      assert(infos.HasMember("code"));
      if(infos["code"] != "Ok"){
        throw custom_exception("OSRM route: "
                               + std::string(infos["message"].GetString()));
      }

      value.AddMember("duration",
                      round_to_distance(infos["routes"][0]["duration"].GetDouble()),
                      allocator);
      value.AddMember("distance",
                      round_to_distance(infos["routes"][0]["distance"].GetDouble()),
                      allocator);
      value.AddMember("geometry",
                      rapidjson::Value(infos["routes"][0]["geometry"], allocator),
                      allocator);
    }
    else{
      // Backward compat.
      assert(infos.HasMember("route_summary"));
      assert(infos["route_summary"].HasMember("total_time"));
      assert(infos["route_summary"].HasMember("total_distance"));
      assert(infos.HasMember("route_geometry"));

      value.AddMember("duration",
                      infos["route_summary"]["total_time"],
                      allocator);
      value.AddMember("distance",
                      infos["route_summary"]["total_distance"],
                      allocator);
      value.AddMember("geometry",
                      rapidjson::Value(infos["route_geometry"], allocator),
                      allocator);
    }
  }
};

#endif
