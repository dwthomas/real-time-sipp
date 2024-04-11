#include <unordered_set>
#include "rtasipp.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"
#include "sippgraph.hpp"

std::unordered_map<Location, double> rtasipp::h_static;
std::unordered_map<const SIPPState<Location> *, rtasipp::CATF> rtasipp::h_dynamic;

inline void dump_h(){
    std::cerr << "h_static\n";
    for(auto acc: rtasipp::h_static){
        std::cerr << acc.first << " " << acc.second << "\n";
    } 
    std::cerr << "h_static done\n";
    std::cerr << "h_dynamic\n";
    for(auto acc: rtasipp::h_dynamic){
        std::cerr << *acc.first << " " << acc.second;
    } 
    std::cerr << "h_dynamic done\n";
}

std::vector<const SIPPState<Location> *> rtasipp::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, double start_time, long expansion_budget){
    std::vector<const SIPPState<Location> *> path;
    struct timespec ts1, ts2;
    m.init();
    auto cur = source;
    double t = start_time;
    while(cur->configuration != dest){
        path.emplace_back(cur);
        asipp::Open open_list;
        clock_gettime(CLOCK_MONOTONIC, &ts1);
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), get_h(*cur, t, dest) , cur, nullptr, nullptr);
        asipp::search_core(g, open_list, dest, m, expansion_budget, &get_h);
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        m.search_time += 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);
        clock_gettime(CLOCK_MONOTONIC, &ts1);
        double h_s_prime = std::numeric_limits<double>::infinity();
        h_dynamic[cur] = CATF();
        for (const auto& node: open_list.queue){
            double h_s_nx = get_h_s(node.state, dest);
            h_s_prime = std::min(h_s_prime, node.g.delta + h_s_nx);
            EdgeATF eatf = node.g;
            eatf.delta += h_s_nx;
            add_h_dyn(cur, eatf);
        }
        if(h_s_prime > get_h_s(cur, dest)){
            set_h_s(cur->configuration, h_s_prime);
        }
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        m.learning_time += 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);

        

        auto e = open_list.top().tla;
        t = e->duration.arrival_time(t);
        cur = e->destination;
    } 
    path.emplace_back(cur);
    std::cout << "Arrival time: " << t << "\n";
    return path;
}