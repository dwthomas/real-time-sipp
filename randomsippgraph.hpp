#pragma once

#include "sippgraph.hpp"
#include "map.hpp"

SippGraph<Location> make_random_sipp_graph(const Map& map, double until,  double occupancy, double min_duration, double max_duration, const Location& start_location, const Location& goal_location, std::size_t seed=0);