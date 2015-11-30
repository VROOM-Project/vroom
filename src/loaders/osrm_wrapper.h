/*
VROOM (Vehicle Routing Open-source Optimization Machine)
Copyright (C) 2015, Julien Coupey

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or (at
your option) any later version.

This program is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OSRM_WRAPPER_H
#define OSRM_WRAPPER_H
#include <vector>
#include <limits>
#include <regex>
#include <boost/asio.hpp>
#include "./problem_io.h"
#include "../structures/matrix.h"
#include "../utils/exceptions.h"

using boost::asio::ip::tcp;

class osrm_wrapper : public problem_io<distance_t>{

private:
  std::string _address;         // OSRM server adress
  std::string _port;            // OSRM server listening port
  std::vector<std::pair<double, double>> _locations;

  std::string build_query(const std::vector<std::pair<double, double>>& locations, 
                          std::string service, 
                          std::string extra_args = "") const{
    // Building query for osrm-routed
    std::string query = "GET /" + service + "?";

    // Adding locations.
    for(auto const& location: locations){
      query += "loc="
        + std::to_string(location.first)
        + ","
        + std::to_string(location.second)
        + "&";
    }

    if(!extra_args.empty()){
      query += extra_args + "&";
    }

    query.pop_back();           // Remove trailing '&'.
    query += " HTTP/1.1\r\n\r\n";
    return query;
  }

  std::string send_then_receive_until(std::string query,
                                      std::string end_str) const{
    std::string response;

    boost::asio::io_service io_service;
    
    tcp::socket s (io_service);
    tcp::resolver r (io_service);
    tcp::resolver::query q(_address, _port);
    
    try{
      boost::asio::connect(s, r.resolve(q));

      boost::asio::write(s, boost::asio::buffer(query));

      boost::asio::streambuf b;
      boost::asio::read_until(s, b, end_str);

      std::istream is(&b);
      std::string line;
      while(std::getline(is, line)){
        response += line;
      }
    }
    catch (boost::system::system_error& e)
      {
        throw custom_exception("failure while connecting to the OSRM server.");
      }
    return response;
  }

  void add_location(const std::string location){
    // Regex check for valid location.
    std::regex valid_loc ("loc=-?[0-9]+\\.?[0-9]*,-?[0-9]+\\.?[0-9]*");
    if(!std::regex_match(location, valid_loc)){
      throw custom_exception("invalid syntax for location "
                             + std::to_string(_locations.size() + 1)
                             + ", see vroom -h for usage display."
                             );
    }

    // Parsing the location is now safe.
    std::size_t separator_rank = location.find(",");
    std::string lat = location.substr(4, separator_rank);
    std::string lon = location.substr(separator_rank + 1, location.length() -1);
    _locations.emplace_back(std::stod(lat, nullptr),
                            std::stod(lon, nullptr));
  }

public:
  osrm_wrapper(std::string address, 
               std::string port,
               std::string loc_input):
    _address(address),
    _port(port){
    // Parsing input in locations.
    std::size_t start = 0;
    std::size_t end = loc_input.find("&", start);
    while(end != std::string::npos){
      this->add_location(loc_input.substr(start, end - start));
      start = end + 1;
      end = loc_input.find("&", start);
    }
    // Adding last element, after last "&".
    end = loc_input.length();
    this->add_location(loc_input.substr(start, end - start));
    
    if(_locations.size() <= 1){
      throw custom_exception("at least two locations required!");
    }
  }

  virtual matrix<distance_t> get_matrix() const override{
    std::string query = this->build_query(_locations, "table");

    std::string response = this->send_then_receive_until(query, "}");

    // Stop at "Bad Request" error from OSRM.
    assert(response.find("Bad Request") == std::string::npos);

    // Removing headers.
    std::string distance_key = "{\"distance_table\":[";
    size_t table_start = response.find(distance_key);
    assert(table_start != std::string::npos);

    size_t offset = table_start + distance_key.size();
    std::string json_content
      = response.substr(offset, response.find("]}") - offset);

    // Parsing json tables to build the matrix.
    std::vector<std::string> lines;
    std::string sep = "],";
    size_t previous_sep = 0;
    size_t current_sep = json_content.find(sep);
    while(current_sep != std::string::npos){
      lines.push_back(json_content.substr(previous_sep + 1,
                                          current_sep - previous_sep - 1));
      previous_sep = current_sep + 2;
      current_sep = json_content.find(sep, current_sep + 1);
    }
    std::string last_line
      = json_content.substr(previous_sep + 1, std::string::npos);
    last_line.pop_back();
    lines.push_back(last_line);
    
    assert(lines.size() == _locations.size());

    matrix<distance_t> m {lines.size()};
    for(std::size_t i = 0; i < lines.size(); ++i){
      sep = ",";
      previous_sep = 0;
      current_sep = lines[i].find(sep);
      std::size_t j = 0;
      while(current_sep != std::string::npos){
        m[i][j] = std::stoul(lines[i].substr(previous_sep,
                                             current_sep - previous_sep));
        ++j;
        previous_sep = current_sep + 1;
        current_sep = lines[i].find(sep, current_sep + 1);
      }
      assert(j == _locations.size() - 1);
      m[i][lines.size() - 1] 
        = std::stoul(lines[i].substr(previous_sep, std::string::npos));
    }

    // Now checking for unfound routes to avoid unexpected behavior
    // (OSRM raises max value for an int).
    std::size_t m_size = m.size();
    std::vector<unsigned> nb_unfound_from_loc (m_size, 0);
    std::vector<unsigned> nb_unfound_to_loc (m_size, 0);
    distance_t unfound_time
      = std::numeric_limits<int>::max();
    
    for(unsigned i = 0; i < m_size; ++i){
      for(unsigned j = 0; j < m_size; ++j){
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
    // Finding the "worst" location.
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

  virtual std::string get_route(const std::list<index_t>& tour) const override{
    std::string result = "\"route\":[";
    for(auto const& step: tour){
      result += "[" + std::to_string(_locations[step].first)
        + "," + std::to_string(_locations[step].second) + "],";
    }
    result.pop_back();          // Remove trailing comma.
    result += "],\"tour\":[";
    for(auto const& step: tour){
      // Using input index to describe locations.
      result += std::to_string(step) + ",";
    }
    result.pop_back();          // Remove trailing comma.
    result += "],";

    return result;
  }

  virtual std::string get_route_geometry(const std::list<index_t>& tour) const override{
    // Ordering locations for the given tour.
    std::vector<std::pair<double, double>> ordered_locations;
    for(auto& step: tour){
      ordered_locations.push_back(_locations[step]);
    }

    std::string query = this->build_query(ordered_locations,
                                          "viaroute",
                                          "uturns=true");

    // Other return status than 0 should have been filtered before
    // with unfound routes check.
    std::string response 
      = this->send_then_receive_until(query, "\"status\":0}");

    // Removing headers
    std::string json_content = response.substr(response.find("{"));

    // Removing extra info
    json_content = json_content.substr(0, 11 + json_content.rfind("\"status\":0}"));

    // Parsing total time/distance and route geometry.
    unsigned time_begin = json_content.find("\"total_time\":");
    unsigned time_end = json_content.find(",", time_begin);
    std::string route_infos = json_content.substr(time_begin,
                                                  time_end - time_begin);
    route_infos += ",";

    unsigned distance_begin = json_content.find("\"total_distance\":");
    unsigned distance_end = json_content.find("}", distance_begin);
    route_infos += json_content.substr(distance_begin,
                                       distance_end - distance_begin);
    route_infos += ",";
    
    unsigned geometry_begin = json_content.find("\"route_geometry\":");
    unsigned geometry_end = json_content.find(",", geometry_begin);
    route_infos += json_content.substr(geometry_begin,
                                       geometry_end - geometry_begin);
    
    return route_infos;
  }
};

#endif
