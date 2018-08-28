#ifndef TIME_WINDOW_H
#define TIME_WINDOW_H

/*

This file is part of VROOM.

Copyright (c) 2015-2018, Julien Coupey.
All rights reserved (see LICENSE).

*/

#include "structures/typedefs.h"

struct time_window_t {
  static const duration_t default_length;
  duration_t start;
  duration_t end;

  // Default "no-constraint" time-window.
  time_window_t();

  time_window_t(duration_t start, duration_t end);

  bool contains(duration_t time) const;

  bool is_default() const;

  friend bool operator<(const time_window_t& lhs, const time_window_t& rhs);
};

#endif
