#include "augmentedsipp.hpp"
#include "structs.hpp"
#include <algorithm>
#include <limits>
#include <utility>
#include <time.h>

using namespace asipp;

std::pair<std::vector<const SIPPState<Location> *>, EdgeATF> asipp::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, double start_time, long expansion_budget, double (*hf)(const SIPPState<Location>&, double , const Location& )){
    Open open_list;
    struct timespec ts1, ts2;
    m.init();
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), start_time, std::numeric_limits<double>::infinity(), 0.0), eightWayDistance(dest, source->configuration), source, nullptr, nullptr);
    auto res =  search_core(g, open_list, dest, m, expansion_budget, hf);
    clock_gettime(CLOCK_MONOTONIC, &ts2);
    m.search_time = 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);
    std::cout << "Arrival time: " << res.second.earliest_arrival_time() << "\n";
    std::cout << "ATF: " << res.second << "\n";
    return res;
}