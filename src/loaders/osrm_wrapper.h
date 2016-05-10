#ifndef OSRM_WRAPPER_H
#define OSRM_WRAPPER_H

/*

This file is part of VROOM.

Copyright (c) 2015-2016, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <vector>
#include <limits>
#include <regex>
#include <boost/asio.hpp>
#include "../../include/rapidjson/error/en.h"
#include "./problem_io.h"
#include "../structures/matrix.h"
#include "../utils/exceptions.h"

using boost::asio::ip::tcp;

class osrm_wrapper : public problem_io<distance_t>{

private:
  enum class LOC_TYPE {START, END, JOB};
  struct Location {LOC_TYPE type; double lat; double lon; index_t job_id;};

  std::string _address;         // OSRM server adress
  std::string _port;            // OSRM server listening port
  std::vector<Location> _locations;

  std::string build_query(const std::vector<Location>& locations, 
                          std::string service, 
                          std::string extra_args = "") const{
    // Building query for osrm-routed
    std::string query = "GET /" + service + "?";

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

    query.pop_back();           // Remove trailing '&'.
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

  void add_location(LOC_TYPE type, const rapidjson::Value& location, index_t job_id = 0){
    if(!location.IsArray()
       or location.Size() != 2
       or !location[0].IsNumber()
       or !location[1].IsNumber()){
      throw custom_exception("Invalid input location");
    }
    _locations.push_back({type, location[0].GetDouble(), location[1].GetDouble(), job_id});
  }

public:
  osrm_wrapper(const std::string& address,
               const std::string& port,
               cl_args_t& cl_args):
    _address(address),
    _port(port){
    rapidjson::Document input;
    std::string error_msg;

    // Parsing input.
    if(input.Parse(cl_args.input.c_str()).HasParseError()){
      std::string error_msg = std::string(rapidjson::GetParseError_En(input.GetParseError()))
        + " (offset: "
        + std::to_string(input.GetErrorOffset())
        + ")";
      throw custom_exception(error_msg);
    }
    
    // Getting vehicle(s).
    if(!input.HasMember("vehicles")
       or !input["vehicles"].IsArray()
       or input["vehicles"].Empty()){
      throw custom_exception("Incorrect vehicles input.");
    }
    if(input["vehicles"].Size() > 1){
      throw custom_exception("Multiple vehicles are not supported (yet).");
    }
    if(!input["vehicles"][0].IsObject()){
      throw custom_exception("Ill-formed vehicle object.");
    }
    bool has_start = input["vehicles"][0].HasMember("start");

    // Check round_trip optional value.
    if(input["vehicles"][0].HasMember("round_trip")
       and !input["vehicles"][0]["round_trip"].IsBool()){
      throw custom_exception("Incorrect round_trip key.");
    }
    // Perform a round trip by default unless "round_trip": false is
    // explicitly specified.
    bool round_trip = !input["vehicles"][0].HasMember("round_trip")
      or input["vehicles"][0]["round_trip"].GetBool();

    if(round_trip and !has_start){
      throw custom_exception("Vehicle start is mandatory for a round trip.");
    }

    if(has_start){
      // Remember the index of the start loc to be added.
      cl_args.start = _locations.size();
      this->add_location(LOC_TYPE::START, input["vehicles"][0]["start"]);
    }

    // Getting jobs.
    if(!input.HasMember("jobs")
       or !input["jobs"].IsArray()){
      throw custom_exception("Incorrect jobs input.");
    }
    for(rapidjson::SizeType i = 0; i < input["jobs"].Size(); ++i){
      if(!input["jobs"][i].IsObject()){
        throw custom_exception("Ill-formed job object.");
      }
      if(!input["jobs"][i].HasMember("location")){
        throw custom_exception("Missing mandatory job location.");
      }
      if(!input["jobs"][i].HasMember("id")){
        throw custom_exception("Missing mandatory job id.");
      }

      this->add_location(LOC_TYPE::JOB,
                         input["jobs"][i]["location"],
                         input["jobs"][i]["id"].GetUint());
    }

    // Add optional vehicle end as last value in _locations.
    bool has_end = input["vehicles"][0].HasMember("end");
    if(round_trip and has_end){
      throw custom_exception("Vehicle end may only be used with round_trip: false.");
    }
    if(!round_trip and !has_start and !has_end){
      throw custom_exception("Vehicle start or end is mandatory with round_trip: false.");
    }
    if(has_end){
      // Remember the index of the end loc to be added.
      cl_args.end = _locations.size();
      this->add_location(LOC_TYPE::END, input["vehicles"][0]["end"]);
    }

    // Deduce forced start and end from input.
    cl_args.force_start = (has_start and !round_trip);
    cl_args.force_end = has_end;

    std::cout << "force_start: " << cl_args.force_start << std::endl;
    std::cout << "force_end: " << cl_args.force_end << std::endl;
    std::cout << "round_trip: " << round_trip << std::endl;
    if(_locations.size() <= 1){
      throw custom_exception("At least two locations required!");
    }
  }

  virtual matrix<distance_t> get_matrix() const override{
    std::string query = this->build_query(_locations, "table");

    std::string response = this->send_then_receive(query);

    // Stop at "Bad Request" error from OSRM.
    assert(response.find("Bad Request") == std::string::npos);

    // Removing headers.
    std::string json_content = response.substr(response.find("{"));

    // Expected matrix size.
    std::size_t m_size = _locations.size();

    // Parsing distance table to build the matrix.
    rapidjson::Document infos;
    assert(!infos.Parse(json_content.c_str()).HasParseError());
    assert(infos.HasMember("distance_table"));
    assert(infos["distance_table"].Size() == m_size);

    // Building matrix and checking for unfound routes to avoid
    // unexpected behavior (OSRM raises max value for an int).
    matrix<distance_t> m {m_size};

    std::vector<unsigned> nb_unfound_from_loc (m_size, 0);
    std::vector<unsigned> nb_unfound_to_loc (m_size, 0);
    distance_t unfound_time
      = std::numeric_limits<int>::max();

    for(rapidjson::SizeType i = 0; i < infos["distance_table"].Size(); ++i){
      const auto& line = infos["distance_table"][i];
      assert(line.Size() == m_size);
      for(rapidjson::SizeType j = 0; j < line.Size(); ++j){
        m[i][j] = line[j].GetUint();
        if(m[i][j] == unfound_time){
          // Just storing info as we don't know yet which location is
          // responsible between i and j.
          ++nb_unfound_from_loc[i];
          ++nb_unfound_to_loc[j];
        }
      }
    }
    
    unsigned max_unfound_routes_for_a_loc = 0;
    index_t error_loc = 0;    // Initial value never actually used.
    std::string error_direction;
    // Finding the "worst" location for unfound routes.
    for(unsigned i = 0; i < m_size; ++i){
      if(nb_unfound_from_loc[i] > max_unfound_routes_for_a_loc){
        max_unfound_routes_for_a_loc = nb_unfound_from_loc[i];
        error_loc = i;
        error_direction = "from";
      }
      if(nb_unfound_to_loc[i] > max_unfound_routes_for_a_loc){
        max_unfound_routes_for_a_loc = nb_unfound_to_loc[i];
        error_loc = i;
        error_direction = "to";
      }
    }
    if(max_unfound_routes_for_a_loc > 0){
      std::string error_msg = "OSRM has unfound route(s) ";
      error_msg += error_direction;
      error_msg += " location at index: ";
      error_msg += std::to_string(error_loc);
      throw custom_exception(error_msg);
    }

    return m;
  }

  virtual void get_steps(const std::list<index_t>& steps,
                         rapidjson::Value& value,
                         rapidjson::Document::AllocatorType& allocator) const override{
    rapidjson::Value steps_array(rapidjson::kArrayType);
    for(auto const& step_id: steps){
      rapidjson::Value json_step(rapidjson::kObjectType);

      // Step type
      json_step.AddMember("type", rapidjson::Value(), allocator);
      switch (_locations[step_id].type){
      case LOC_TYPE::START:
        json_step["type"].SetString("start");
        break;
      case LOC_TYPE::END:
        json_step["type"].SetString("end");
        break;
      case LOC_TYPE::JOB:
        json_step["type"].SetString("job");
        break;
      }

      // Location coordinates.
      json_step.AddMember("location",
                           rapidjson::Value(rapidjson::kArrayType).Move(),
                           allocator);
      json_step["location"].PushBack(_locations[step_id].lat, allocator);
      json_step["location"].PushBack(_locations[step_id].lon, allocator);

      if(_locations[step_id].type == LOC_TYPE::JOB){
        json_step.AddMember("job", _locations[step_id].job_id, allocator);
      }

      steps_array.PushBack(json_step, allocator);
    }
    value.Swap(steps_array);
  }

  virtual void get_route_infos(const std::list<index_t>& steps,
                               rapidjson::Value& value,
                               rapidjson::Document::AllocatorType& allocator) const override{
    // Ordering locations for the given steps.
    std::vector<Location> ordered_locations;
    for(auto& step: steps){
      ordered_locations.push_back(_locations[step]);
    }

    std::string query = this->build_query(ordered_locations,
                                          "viaroute",
                                          "alt=false&uturns=true");
    std::string response = this->send_then_receive(query);

    // Removing headers
    std::string json_content = response.substr(response.find("{"));

    // Parsing total time/distance and route geometry.
    rapidjson::Document infos;
    // FIXME: use exceptions?
    assert(!infos.Parse(json_content.c_str()).HasParseError());
    assert(infos.HasMember("route_summary"));
    assert(infos["route_summary"].HasMember("total_time"));
    assert(infos["route_summary"].HasMember("total_distance"));
    assert(infos.HasMember("route_geometry"));

    value.AddMember("geometry",
                    rapidjson::Value(infos["route_geometry"], allocator),
                    allocator);
    value.AddMember("duration",
                    infos["route_summary"]["total_time"],
                    allocator);
    value.AddMember("distance",
                    infos["route_summary"]["total_distance"],
                    allocator);
  }
};

#endif
