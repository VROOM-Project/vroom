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

local_search::local_search(const tsp& problem,
                           const std::list<index_t>& tour,
                           bool verbose):
  _matrix(problem.get_matrix()),
  _symmetric_matrix(problem.is_symmetric()),
  _verbose(verbose) 
{
  auto location = tour.cbegin();
  index_t first_index = *location;
  index_t current_index = first_index;
  index_t last_index = first_index;
  ++location;
  while(location != tour.cend()){
    current_index = *location;
    _edges.emplace(last_index, current_index);
    last_index = current_index;
    ++location;
  }
  _edges.emplace(last_index, first_index);
}

distance_t local_search::relocate_step(){
  distance_t gain = 0;
  bool amelioration_found = false;
  for(auto edge_1 = _edges.cbegin(); edge_1 != _edges.cend(); ++edge_1){
    // Going through the tour while checking for insertion of
    // edge_1->second between two other nodes (edge_2_*).
    //
    // Namely edge_1->first --> edge_1->second --> next is replaced by
    // edge_1->first --> next while edge_2->first --> edge_2->second is
    // replaced by edge_2->first --> edge_1->second --> edge_2->second.
    index_t next = _edges.at(edge_1->second);
    index_t relocated_node = edge_1->second;

    // Precomputing weights not depending on edge_2.
    distance_t first_potential_add = _matrix[edge_1->first][next];
    distance_t edge_1_weight = _matrix[edge_1->first][edge_1->second];
    distance_t relocated_next_weight = _matrix[relocated_node][next];

    for(auto edge_2 = _edges.cbegin(); edge_2 != _edges.cend(); ++edge_2){
      if((edge_2 == edge_1) or (edge_2->first == relocated_node)){
        continue;
      }
      distance_t before_cost
        = edge_1_weight
        + relocated_next_weight
        + _matrix[edge_2->first][edge_2->second];
      distance_t after_cost
        = first_potential_add
        + _matrix[edge_2->first][relocated_node]
        + _matrix[relocated_node][edge_2->second];

      if(before_cost > after_cost){
        amelioration_found = true;
        gain = before_cost - after_cost;

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

distance_t local_search::perform_all_relocate_steps(){
  distance_t total_gain = 0;
  unsigned relocate_iter = 0;
  distance_t gain = 0;
  do{
    gain = this->relocate_step();

    if(gain > 0){
      total_gain += gain;
      ++relocate_iter;
    }
  } while(gain > 0);

  if(_verbose){
    std::cout << "Performed "
              << relocate_iter << " \"relocate\" steps, gaining "
              << total_gain
              << std::endl;
  }
  return total_gain;
}

distance_t local_search::avoid_loop_step(){
  // In some cases, the solution can contain "loops" that other
  // operators can't fix. Those are found with two steps:
  // 
  // 1) searching for all nodes that can be relocated somewhere else
  // AT NO COST because they are already on some other way.
  // 
  // 2) listing all "chains" of two or mode consecutive such nodes.
  // 
  // Starting from the longest such chain, the fix is to:
  // 
  // 3) relocate all nodes along the chain until an amelioration pops
  // out, meaning a "loop" has been undone.

  distance_t gain = 0;

  // Going through all candidate nodes for relocation.
  index_t previous_candidate = 0;
  index_t candidate = _edges.at(previous_candidate);

  // Remember previous steps for each node, required for step 3.
  std::map<index_t, index_t> previous;
  previous.emplace(candidate, previous_candidate);

  // Storing chains as described in 2.
  std::vector<std::list<index_t>> relocatable_chains;
  std::list<index_t> current_relocatable_chain;

  // Remember possible position for further relocation of candidate
  // nodes.
  std::map<index_t, index_t> possible_position;

  do{
    index_t current = _edges.at(candidate);
        
    bool candidate_relocatable = false;
    while((current != previous_candidate) and !candidate_relocatable){
      index_t next = _edges.at(current);
      if((_matrix[current][candidate] + _matrix[candidate][next]
          <= _matrix[current][next])
         // Relocation at no cost.
         and (_matrix[current][candidate] > 0)
         // Set aside the case of identical locations.
         and (_matrix[candidate][next] > 0)){
        candidate_relocatable = true;
        // Remember possible relocate position for candidate.
        possible_position.emplace(candidate, current);
      }
      current = next;
    }
    if(candidate_relocatable){
      current_relocatable_chain.push_back(candidate);
    }
    else{
      if(current_relocatable_chain.size() > 1){
        relocatable_chains.push_back(current_relocatable_chain);
      }
      current_relocatable_chain.clear();
    }
    previous_candidate = candidate;
    candidate = _edges.at(candidate);
    previous.emplace(candidate, previous_candidate);
  }while(candidate != 0);

  // Reorder to try the longest chains first.
  std::sort(relocatable_chains.begin(),
            relocatable_chains.end(),
            [](const auto& lhs, const auto& rhs){
              return lhs.size() > rhs.size();
            });

  bool amelioration_found = false;
  for(auto const& chain: relocatable_chains){
    // Going through step 3. for all chains by decreasing length.
    distance_t before_cost = 0;
    distance_t after_cost = 0;

    // Work on copies as modifications are needed while going through
    // the chain.
    std::map<index_t, index_t> edges_c = _edges;
    std::map<index_t, index_t> previous_c = previous;

    for(auto const& step: chain){
      // Compare situations to see if relocating current step after
      // possible_position.at(step) will decrease overall cost.
      // 
      // Situation before:
      // 
      // previous_c.at(step)-->step-->edges_c.at(step)
      // possible_position.at(step)-->edges_c.at(possible_position.at(step))
      // 
      // Situation after: 
      // 
      // previous_c.at(step)-->edges_c.at(step)
      // possible_position.at(step)-->step-->edges_c.at(possible_position.at(step))

      before_cost += _matrix[previous_c.at(step)][step];
      before_cost += _matrix[step][edges_c.at(step)];
      after_cost += _matrix[previous_c.at(step)][edges_c.at(step)];
      before_cost 
        += _matrix[possible_position.at(step)][edges_c.at(possible_position.at(step))];
      after_cost += _matrix[possible_position.at(step)][step];
      after_cost += _matrix[step][edges_c.at(possible_position.at(step))];

      // Linking previous_c.at(step) with edges_c.at(step) in both
      // ways as remembering previous nodes is required.
      previous_c.at(edges_c.at(step)) = previous_c.at(step);
      edges_c.at(previous_c.at(step)) = edges_c.at(step);

      // Relocating step between possible_position.at(step) and
      // edges_c.at(possible_position.at(step)) in both ways too.
      edges_c.at(step) = edges_c.at(possible_position.at(step));
      previous_c.at(edges_c.at(possible_position.at(step))) = step;

      edges_c.at(possible_position.at(step)) = step;
      previous_c.at(step) = possible_position.at(step);
      
      if(before_cost > after_cost){
        amelioration_found = true;
        gain = before_cost - after_cost;
        _edges.swap(edges_c);   // Keep changes.
        break;
      }
    }
    if(amelioration_found){
      break;
    }
  }

  return gain;
}

distance_t local_search::perform_all_avoid_loop_steps(){
  distance_t total_gain = 0;
  unsigned relocate_iter = 0;
  distance_t gain = 0;
  do{
    gain = this->avoid_loop_step();

    if(gain > 0){
      total_gain += gain;
      ++relocate_iter;
    }
  } while(gain > 0);

  if(_verbose){
    std::cout << "Performed "
              << relocate_iter << " \"avoid loop\" steps, gaining "
              << total_gain
              << std::endl;
  }
  return total_gain;
}


distance_t local_search::two_opt_step(){
  distance_t gain = 0;
  for(auto edge_1 = _edges.cbegin(); edge_1 != _edges.cend(); ++edge_1){
    auto edge_2 = edge_1;
    ++edge_2;
    // Trying to improve two "crossing edges".
    //
    // Namely edge_1->first --> edge_1->second and edge_2->fist -->
    // edge_2->second are replaced by edge_1->first --> edge_2->first
    // and edge_1->second --> edge_2->second. The tour between
    // edge_2->first and edge_1->second need to be reversed.
    bool amelioration_found = false;
    for(; edge_2 != _edges.cend(); ++edge_2){
      if(edge_2->first == edge_1->second){
        continue;
      }
      distance_t before_cost
        = _matrix[edge_1->first][edge_1->second]
        + _matrix[edge_2->first][edge_2->second];
      distance_t after_cost
        = _matrix[edge_1->first][edge_2->first]
        + _matrix[edge_1->second][edge_2->second];
      if(!_symmetric_matrix){
        // Adding part of the tour that needs to be reversed.
        for(index_t current = edge_1->second;
            current != edge_2->first;
            current = _edges.at(current)){
          before_cost += _matrix[current][_edges.at(current)];
          after_cost += _matrix[_edges.at(current)][current];
        }
      }

      if(before_cost > after_cost){
        amelioration_found = true;
        gain = before_cost - after_cost;

        // Storing part of the tour that needs to be reversed.
        std::list<index_t> to_reverse;
        for(index_t current = edge_1->second;
            current != edge_2->first;
            current = _edges.at(current)){
          to_reverse.push_back(current);
        }
        // Performing exchange.
        index_t current = edge_2->first;
        index_t last = edge_2->second;
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

distance_t local_search::perform_all_two_opt_steps(){
  distance_t total_gain = 0;
  unsigned two_opt_iter = 0;
  distance_t gain = 0;
  do{
    gain = this->two_opt_step();

    if(gain > 0){
      total_gain += gain;
      ++two_opt_iter;
    }
  } while(gain > 0);

  if(_verbose){
    std::cout << "Performed "
              << two_opt_iter << " \"2-opt\" steps, gaining "
              << total_gain
              << std::endl;
  }
  return total_gain;
}

distance_t local_search::or_opt_step(){
  distance_t gain = 0;
  bool amelioration_found = false;
  for(auto edge_1 = _edges.cbegin(); edge_1 != _edges.cend(); ++edge_1){
    // Going through the tour while checking the move of edge after
    // edge_1 in place of another edge (edge_2).
    //
    // Namely edge_1->first --> edge_1->second --> next --> next_2 is
    // replaced by edge_1->first --> next_2 while edge_2->first -->
    // edge_2->second is replaced by edge_2->first --> edge_1->second
    // --> next --> edge_2->second.
    index_t first_relocated = edge_1->second;
    index_t next = _edges.at(edge_1->second);
    index_t next_2 = _edges.at(next);

    // Precomputing weights not depending on edge_2.
    distance_t first_potential_add = _matrix[edge_1->first][next_2];
    distance_t edge_1_weight = _matrix[edge_1->first][edge_1->second];
    distance_t next_next_2_weight = _matrix[next][next_2];

    for(auto edge_2 = _edges.cbegin(); edge_2 != _edges.cend(); ++edge_2){
      if((edge_2 == edge_1)
         or (edge_2->first == first_relocated)
         or (edge_2->first == next)){
        continue;
      }
      distance_t before_cost
        = edge_1_weight
        + next_next_2_weight
        + _matrix[edge_2->first][edge_2->second];
      distance_t after_cost
        = first_potential_add
        + _matrix[edge_2->first][first_relocated]
        + _matrix[next][edge_2->second];
      if(before_cost > after_cost){
        amelioration_found = true;
        gain = before_cost - after_cost;

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

distance_t local_search::perform_all_or_opt_steps(){
  distance_t total_gain = 0;
  unsigned or_opt_iter = 0;
  distance_t gain = 0;
  do{
    gain = this->or_opt_step();
    if(gain > 0){
      total_gain += gain;
      ++or_opt_iter;
    }
  } while(gain > 0);

  if(_verbose){
    std::cout << "Performed "
              << or_opt_iter << " \"or_opt\" steps, gaining "
              << total_gain
              << std::endl;
  }
  return total_gain;
}

std::list<index_t> local_search::get_tour(index_t first_index) const{
  std::list<index_t> tour;
  tour.push_back(first_index);
  index_t next_index = _edges.at(first_index);
  while(next_index != first_index){
    tour.push_back(next_index);
    next_index = _edges.at(next_index);
  }
  return tour;
}
