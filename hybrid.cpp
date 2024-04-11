#include <unordered_set>
#include "plrtosipphonly.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"
#include "newatsippgraph.hpp"
#include "sippgraph.hpp"
#include "rtasipp.hpp"
#include "hybrid.hpp"

std::vector<const SIPPState<Location> *> hybrid::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, long budget, double start_time){
    std::vector<const SIPPState<Location> *> path;
    struct timespec ts1, ts2;
    m.init();
    auto cur = source;
    double t = start_time;
    while(cur->configuration != dest){
        path.emplace_back(cur);
        asipp::Open open_list;
        clock_gettime(CLOCK_MONOTONIC, &ts1);
        open_list.emplace(EdgeATF(-std::numeric_limits<double>::infinity(), t, std::numeric_limits<double>::infinity(), 0.0), rtasipp::get_h(*cur, t, dest) , cur, nullptr, nullptr);
        asipp::search_core(g, open_list, dest, m, budget, rtasipp::get_h);
        clock_gettime(CLOCK_MONOTONIC, &ts2);
        m.search_time += 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);
        if(open_list.empty()){
            std::cerr << "No path found\n";
            exit(-1);
        }
        clock_gettime(CLOCK_MONOTONIC, &ts1);

        plrtosipphonly::lsslrtsipp(g, open_list, dest, m);
        rtasipp::h_dynamic[cur] = rtasipp::CATF();

        double h_s_prime = std::numeric_limits<double>::infinity();
        for (const auto& node: open_list.queue){
            double h_s_nx = rtasipp::get_h_s(node.state->configuration, dest);
            h_s_prime = std::min(h_s_prime, node.g.delta + h_s_nx);
            EdgeATF eatf = node.g;
            eatf.delta += h_s_nx;
            rtasipp::add_h_dyn(cur, eatf);
        }
        if(h_s_prime > rtasipp::get_h_s(cur, dest)){
            rtasipp::set_h_s(cur->configuration, h_s_prime);
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