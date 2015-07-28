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
#include <cstring>              // c_str()
#include<sys/socket.h>          //socket
#include<arpa/inet.h>           //inet_addr
// #include<netdb.h>               //hostent
#include "./matrix_loader.h"
#include "../structures/matrix.h"

class osrm_wrapper : public matrix_loader<distance_t, double>{

private:
  int _sock;                    // socket
  std::string _address;  // OSRM server adress
  int _port;             // OSRM server listening port
  struct sockaddr_in _server;   // server

  // Perform socket connection.
  bool osrm_connect(){
    // Create socket if necessary.
    if(_sock == -1){
      //Create socket
      _sock = socket(AF_INET, SOCK_STREAM, 0);
      if (_sock == -1){
        std::cout << "Could not create socket\n";
        exit(0);
      }
    }

    // Using plain IP address (static member).
    _server.sin_addr.s_addr = inet_addr(_address.c_str());
  
    _server.sin_family = AF_INET;
    _server.sin_port = htons(_port);

    // Connect to osrm-routed server.
    if (connect(_sock , (struct sockaddr *)&_server , sizeof(_server)) < 0){
      std::cout << "Connect to the OSRM server failed!\n";
      exit(0);
    }
    return true;
  }
  
  // Send a request or osrm routing deamon.
  bool send_data(std::string data){
    if(send(_sock, data.c_str(), strlen(data.c_str()), 0) < 0){
      std::cout << "Send to OSRM server failed!\n";
      exit(0);
    }
    return true;
  }

  // Receive given amount of data.
  std::string receive(const int size=512){
    char* buffer = new char[size];
     
    // Receive a reply from the server.
    if(recv(_sock, buffer, size, 0) < 0){
      std::cout << "Receiving from OSRM server failed!\n";
      exit(0);
    }
     
    std::string reply (buffer, size);
    delete[] buffer;
    return reply;
  }

public:
  osrm_wrapper(std::string address, int port):
    _sock(-1),
    _address(address),
    _port(port){
    // Connect to osrm-routed.
    this->osrm_connect();
  }

  virtual matrix<distance_t> load_matrix(const std::vector<std::pair<double, double>>& places){
    // Building query for osrm-routed
    std::string query = "GET /table?";

    // Adding places.
    for(auto place = places.cbegin(); place != places.cend(); ++place){
      query += "&loc="
        + std::to_string(place->first)
        + ","
        + std::to_string(place->second);
    }

    query += " HTTP/1.1\r\n\r\n";

    // Send query.
    this->send_data(query);
     
    std::string response, buffer;
    int buffer_size = 126;
    // First reading
    response = this->receive(buffer_size);

    // Storing current position to avoid unecessary search for the end
    // string.
    std::size_t position = 0;
    std::string end_str = "]]}";
  
    while(response.find(end_str, position) == std::string::npos){
      // End of response not yet received
      buffer = this->receive(buffer_size);
      if(buffer.find("Bad Request") != std::string::npos){
        // Problem with the OSRM request, encountered for exemple in
        // cases with more than 342 locs.
        std::cout << "Bad Request response from OSRM, too much localisations?\n";
        exit(0);
      }
      response += buffer;
      // To be able to find the end string even if truncated between
      // two buffer reception.
      position += buffer_size - 3;
    }

    // Removing headers.
    std::string distance_key = "{\"distance_table\":[";
    size_t table_start = response.find(distance_key);
    if(table_start == std::string::npos){
      std::cout << "Unexpected form of OSRM return!\n";
      exit(0);
    }

    size_t offset = table_start + distance_key.size();
    std::string json_content
      = response.substr(offset, response.find("]}") - offset);

    // Parsing json tables to build the matrix from a vector.
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

    // for(auto line = lines.cbegin(); line != lines.cend(); ++line){
    //   std::cout << *line << std::endl;
    // }
    
    std::vector<std::vector<distance_t>> matrix_as_vector;
    for(auto line = lines.cbegin(); line != lines.cend(); ++line){
      std::vector<distance_t> line_as_vector;
      sep = ",";
      previous_sep = 0;
      current_sep = line->find(sep);
      while(current_sep != std::string::npos){
        distance_t current_value
          = std::stoul(line->substr(previous_sep,
                                    current_sep - previous_sep));
        line_as_vector.push_back(current_value);
        previous_sep = current_sep + 1;
        current_sep = line->find(sep, current_sep + 1);
      }
      distance_t last_value
        = std::stoul(line->substr(previous_sep, std::string::npos));
      line_as_vector.push_back(last_value);
      
      matrix_as_vector.push_back(line_as_vector);
    }

    matrix<distance_t> m (matrix_as_vector);
    // m.print();

    // Now checking for unfound routes to avoid unexpected behavior
    // (OSRM raises max value for an int).
    unsigned m_size = m.size();
    std::vector<unsigned> nb_unfound_from_loc (m_size, 0);
    std::vector<unsigned> nb_unfound_to_loc (m_size, 0);
    distance_t unfound_time
      = std::numeric_limits<int>::max();
    
    for(unsigned i = 0; i < m_size; ++i){
      for(unsigned j = 0; j < m_size; ++j){
        if(m(i, j) == unfound_time){
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
      std::cout << "OSRM has unfound route(s) "
                << error_direction
                << " location at index: "
                << std::to_string(error_loc)
                << std::endl;
      exit(0);      
    }

    return m;
  }
};

#endif
