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

#include "local_search.h"

local_search::local_search(tsp* problem, std::list<unsigned> tour):
  _problem(problem),
  _matrix(_problem->get_matrix()){
  auto place = tour.cbegin();
  unsigned first_index = *place;
  unsigned current_index = first_index;
  unsigned last_index = first_index;
  ++place;
  while(place != tour.cend()){
    current_index = *place;
    _edges.emplace(last_index, current_index);
    last_index = current_index;
    ++place;
  }
  _edges.emplace(last_index, first_index);
}

unsigned local_search::relocate_step(){
  unsigned gain = 0;
  bool amelioration_found = false;
  for(auto edge_1 = _edges.cbegin(); edge_1 != _edges.cend(); ++edge_1){
    // Going through the tour while checking for insertion of
    // edge_1->second between two other nodes (edge_2_*).
    //
    // Namely edge_1->first --> edge_1->second --> next is replaced by
    // edge_1->first --> next while edge_2->first --> edge_2->second is
    // replaced by edge_2->first --> edge_1->second --> edge_2->second.
    unsigned next = _edges.at(edge_1->second);
    unsigned relocated_node = edge_1->second;

    // Precomputing weights not depending on edge_2.
    unsigned first_potential_add = _matrix(edge_1->first, next);
    unsigned edge_1_weight = _matrix(edge_1->first, edge_1->second);
    unsigned relocated_next_weight = _matrix(relocated_node, next);

    for(auto edge_2 = _edges.cbegin(); edge_2 != _edges.cend(); ++edge_2){
      if((edge_2 == edge_1) or (edge_2->first == relocated_node)){
        continue;
      }
      int current_diff = edge_1_weight
        + relocated_next_weight
        + _matrix(edge_2->first, edge_2->second)
        - first_potential_add
        - _matrix(edge_2->first, relocated_node)
        - _matrix(relocated_node, edge_2->second);
      if(current_diff > 0){
        amelioration_found = true;
        gain = current_diff;
        // std::cout << "Gain:" << current_diff << std::endl;
        // std::cout << edge_1->first
        //           << "-" << _matrix(edge_1->first, edge_1->second) << "->"
        //           << edge_1->second
        //           << "-" << _matrix(edge_1->second, next) << "->"
        //           << next
        //           << std::endl;
        // std::cout << edge_2->first
        //           << "-" << _matrix(edge_2->first, edge_2->second) << "->"
        //           << edge_2->second
        //           << std::endl;

        // Performing exchange.
        _edges.at(edge_1->first) = next;
        _edges.at(relocated_node) = edge_2->second;
        _edges.at(edge_2->first) = relocated_node;
        break;
      }
    }
    if(amelioration_found){
      break;
    }
  }

  return gain;
}

unsigned local_search::perform_all_relocate_steps(){
  unsigned total_gain = 0;
  unsigned relocate_iter = 0;
  unsigned gain = 0;
  do{
    gain = this->relocate_step();
    total_gain += gain;

    if(gain > 0){
      ++relocate_iter;
      // logger log ("relocate.json");
      // log.tour_to_file(*_problem, this->get_tour(0), 0);
    }
  } while(gain > 0);

  std::cout << "Performed "
            << relocate_iter << " \"relocate\" steps, gaining "
            << total_gain
            << std::endl;
  return total_gain;
}

unsigned local_search::two_opt_step(){
  unsigned gain = 0;
  for(auto edge_1 = _edges.cbegin(); edge_1 != _edges.cend(); ++edge_1){
    auto edge_2 = edge_1;
    ++edge_2;
    bool amelioration_found = false;
    for(; edge_2 != _edges.cend(); ++edge_2){
      int current_diff = _matrix(edge_1->first, edge_1->second)
        + _matrix(edge_2->first, edge_2->second)
        - _matrix(edge_1->first, edge_2->first)
        - _matrix(edge_1->second, edge_2->second);
      if(current_diff > 0){
        amelioration_found = true;
        gain = current_diff;
        // std::cout << "Gain:" << current_diff << std::endl;
        // std::cout << edge_1->first
        //           << "-" << _matrix(edge_1->first, edge_1->second) << "->"
        //           << edge_1->second
        //           << std::endl;
        // std::cout << edge_2->first
        //           << "-" << _matrix(edge_2->first, edge_2->second) << "->"
        //           << edge_2->second
        //           << std::endl;

        // Storing part of the tour that needs to be reversed.
        std::list<unsigned> to_reverse;
        for(unsigned current = edge_1->second;
            current != edge_2->first;
            current = _edges.at(current)){
          to_reverse.push_back(current);
        }
        // Performing exchange.
        unsigned current = edge_2->first;
        unsigned last = edge_2->second;
        _edges.at(edge_1->first) = current;
        for(auto next = to_reverse.rbegin(); next != to_reverse.rend(); ++next){
          _edges.at(current) = *next;
          current = *next;
       }
        _edges.at(current) = last;
        break;
      }
    }
    if(amelioration_found){
      break;
    }
  }

  return gain;
}

unsigned local_search::perform_all_two_opt_steps(){
  unsigned total_gain = 0;
  unsigned two_opt_iter = 0;
  unsigned gain = 0;
  do{
    gain = this->two_opt_step();
    total_gain += gain;

    if(gain > 0){
      ++two_opt_iter;
      // logger log ("two_opt.json");
      // log.tour_to_file(*_problem, this->get_tour(0), 0);
    }
  } while(gain > 0);

  std::cout << "Performed "
            << two_opt_iter << " \"2-opt\" steps, gaining "
            << total_gain
            << std::endl;
  return total_gain;
}

unsigned local_search::or_opt_step(){
  unsigned gain = 0;
  bool amelioration_found = false;
  for(auto edge_1 = _edges.cbegin(); edge_1 != _edges.cend(); ++edge_1){
    // Going through the tour while checking the move of edge after
    // edge_1 in place of another edge (edge_2).
    //
    // Namely edge_1->first --> edge_1->second --> next --> next_2 is
    // replaced by edge_1->first --> next_2 while edge_2->first -->
    // edge_2->second is replaced by edge_2->first --> edge_1->second
    // --> next --> edge_2->second.
    unsigned first_relocated = edge_1->second;
    unsigned next = _edges.at(edge_1->second);
    unsigned next_2 = _edges.at(next);

    // Precomputing weights not depending on edge_2.
    unsigned first_potential_add = _matrix(edge_1->first, next_2);
    unsigned edge_1_weight = _matrix(edge_1->first, edge_1->second);
    unsigned next_next_2_weight = _matrix(next, next_2);

    for(auto edge_2 = _edges.cbegin(); edge_2 != _edges.cend(); ++edge_2){
      if((edge_2 == edge_1)
         or (edge_2->first == first_relocated)
         or (edge_2->first == next)){
        continue;
      }
      int current_diff = edge_1_weight
        + next_next_2_weight
        + _matrix(edge_2->first, edge_2->second)
        - first_potential_add
        - _matrix(edge_2->first, first_relocated)
        - _matrix(next, edge_2->second);
      if(current_diff > 0){
        amelioration_found = true;
        gain = current_diff;
        // std::cout << "Gain:" << current_diff << std::endl;
        // std::cout << edge_1->first
        //           << "-" << edge_1_weight << "->"
        //           << first_relocated
        //           << " / "
        //           << next << "->" << next_next_2_weight << "->"
        //           << next_2
        //           << std::endl;
        // std::cout << edge_2->first
        //           << "-" << _matrix(edge_2->first, edge_2->second) << "->"
        //           << edge_2->second
        //           << std::endl;

        // Performing exchange.
        _edges.at(edge_1->first) = next_2;
        _edges.at(next) = edge_2->second;
        _edges.at(edge_2->first) = first_relocated;
        break;
      }
    }
    if(amelioration_found){
      break;
    }
  }

  return gain;
}

unsigned local_search::perform_all_or_opt_steps(){
  unsigned total_gain = 0;
  unsigned or_opt_iter = 0;
  unsigned gain = 0;
  do{
    gain = this->or_opt_step();
    total_gain += gain;

    if(gain > 0){
      ++or_opt_iter;
      // logger log ("or_opt.json");
      // log.tour_to_file(*_problem, this->get_tour(0), 0);
    }
  } while(gain > 0);

  std::cout << "Performed "
            << or_opt_iter << " \"or_opt\" steps, gaining "
            << total_gain
            << std::endl;
  return total_gain;
}

std::list<unsigned> local_search::get_tour(unsigned first_index) const{
  std::list<unsigned> tour;
  tour.push_back(first_index);
  unsigned next_index = _edges.at(first_index);
  while(next_index != first_index){
    tour.push_back(next_index);
    next_index = _edges.at(next_index);
  }
  return tour;
}
