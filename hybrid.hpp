#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include "atsippgraph.hpp"
#include "augmentedsipp.hpp"
#include "sippgraph.hpp"
#include "structs.hpp"

namespace hybrid{
    std::vector<const SIPPState<Location> *> search(const AtSippGraph<Location> & g, const SIPPState<Location> * source, const Location& dest, MetaData & m, long budget, double start_time = 0.0);
}

