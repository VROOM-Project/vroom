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
#include <boost/asio.hpp>
#include "./matrix_loader.h"
#include "../structures/matrix.h"
#include "../utils/exceptions.h"

using boost::asio::ip::tcp;

class osrm_wrapper : public matrix_loader<distance_t, double>{

private:
  std::string _address;         // OSRM server adress
  std::string _port;            // OSRM server listening port

  std::string build_query(const std::vector<std::pair<double, double>>& locations, std::string service){
    // Building query for osrm-routed
    std::string query = "POST /" + service + "?";

    // Adding locations.
    for(auto const& location: locations){
      query += "loc="
        + std::to_string(location.first)
        + ","
        + std::to_string(location.second)
        + "&";
    }

    query.pop_back();           // Remove trailing '&'.
    query += " HTTP/1.1\r\n\r\n";
    return query;
  }

  std::string send_then_receive_until(std::string query,
                                      std::string end_str){
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

public:
  osrm_wrapper(std::string address, std::string port):
    _address(address),
    _port(port){
  }

  virtual matrix<distance_t> load_matrix(const std::vector<std::pair<double, double>>& locations) override{
    std::string query = this->build_query(locations, "table");

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

    matrix<distance_t> m {lines.size()};
    for(std::size_t i = 0; i < lines.size(); ++i){
      sep = ",";
      previous_sep = 0;
      current_sep = lines[i].find(sep);
      std::size_t j = 0;
      while(current_sep != std::string::npos){
        assert(j < lines.size());
        m[i][j] = std::stoul(lines[i].substr(previous_sep,
                                             current_sep - previous_sep));
        ++j;
        previous_sep = current_sep + 1;
        current_sep = lines[i].find(sep, current_sep + 1);
      }
      m[i][lines.size()-1] 
        = std::stoul(lines[i].substr(previous_sep, std::string::npos));
    }

    // m.print();

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

  std::string viaroute_summary(const std::vector<std::pair<double, double>>& locations){
    std::string query = this->build_query(locations, "viaroute");

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
