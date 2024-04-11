#include "plrtosipphonly.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"
#include "rtasipp.hpp"
#include "newatsippgraph.hpp"
#include "sippgraph.hpp"

void plrtosipphonly::lsslrtsipp(const AtSippGraph<Location>& g, const asipp::Open& open_list, const Location& dest, MetaData& m){
    std::unordered_map<Location, double> h_s_prime;
    auto closed = open_list.expanded; 
    LSSOpen dijkstraOpen;
    for(auto n: open_list.queue){
        auto h_s = rtasipp::get_h_s(n.state, dest);
        h_s_prime[n.state->configuration] = h_s;
        dijkstraOpen.emplace(h_s, n.state);
    }
    while(!dijkstraOpen.empty() && !closed.empty()){
        auto n = dijkstraOpen.top();
        m.learn_expanded++;
        dijkstraOpen.pop();
        closed.erase(n.node);
        auto n_loc = n.node->configuration;
        double h_s_n = h_s_prime[n_loc];
        for (const auto& p: g.predecessors.at(n.node)){
            if(closed.find(p.source) == closed.end()){
                continue;
            }
            auto p_loc = p.source->configuration;
            auto i = h_s_prime.find(p_loc);
            if(i == h_s_prime.end()){
                h_s_prime[p_loc] = std::numeric_limits<double>::infinity();
            }
            double h_s_p = h_s_prime[p_loc];
            double edge_cost =  eightWayDistance(p_loc, n_loc);
            if(h_s_p > edge_cost + h_s_n){
                h_s_prime[p_loc] = edge_cost + h_s_n;
                dijkstraOpen.emplace(edge_cost + h_s_n, p.source);
            }
        }
    }
    for(auto x: h_s_prime){
        rtasipp::set_h_s(x.first, std::max(rtasipp::get_h_s(x.first, dest), x.second));
    }
} 



std::vector<const SIPPState<Location> *> plrtosipphonly::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, long budget, double start_time){
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
        lsslrtsipp(g, open_list, dest, m);
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