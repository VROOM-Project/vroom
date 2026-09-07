#ifndef TASK_GROUP_H
#define TASK_GROUP_H

/*

This file is part of VROOM.

Copyright (c) 2015-2025, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include <string>
#include <vector>

#include "structures/typedefs.h"

namespace vroom {

// A set of tasks of which at most max_tasks may be assigned in a
// solution. Typical use: tasks competing for a resource the company
// only has a limited number of, so that the solver decides which of
// them are left unassigned.
struct TaskGroup {
  const Id id;
  const unsigned max_tasks;
  const std::string description;
  // Ranks (in Input::jobs) of the member tasks, filled by
  // Input::add_job and Input::add_shipment. A shipment is a single
  // member, stored as the rank of its pickup.
  std::vector<Index> tasks;

  TaskGroup(Id id, unsigned max_tasks, std::string description = "")
    : id(id), max_tasks(max_tasks), description(std::move(description)) {
  }
};

} // namespace vroom

#endif
