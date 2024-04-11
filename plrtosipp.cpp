#include <unordered_set>
#include "plrtosipp.hpp"
#include "atf.hpp"
#include "augmentedsipp.hpp"
#include "newatsippgraph.hpp"
#include "plrtosipphonly.hpp"
#include "sippgraph.hpp"
#include "rtasipp.hpp"

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


void plrtosipp::plrtolearn(const AtSippGraph<Location>& g, const asipp::Open& open_list, const Location& dest, MetaData& m){
    auto closed = open_list.expanded; 
    for (const auto& s: closed){ // node in closed
        rtasipp::h_dynamic[s.first] = rtasipp::CATF();   
    }
    plrtosipphonly::LSSOpen dijkstraOpen;
    for (const auto& s: open_list.queue){
        double h = rtasipp::get_h_s(s.state->configuration, dest);
        rtasipp::h_dynamic[s.state] = rtasipp::CATF();
        rtasipp::h_dynamic[s.state].insert(shiftIdentity(h), nullptr);
        dijkstraOpen.emplace(rtasipp::h_dynamic[s.state].earliest_arrival_time(), s.state);
    }
    while(!dijkstraOpen.empty() && !closed.empty()){
        auto n = dijkstraOpen.top();
        m.learn_expanded++;
        dijkstraOpen.pop();
        closed.erase(n.node);
        auto h_d_n = rtasipp::h_dynamic[n.node];
        for (auto e: g.predecessors.at(n.node)){
            if(closed.find(e.source) == closed.end()){
                continue;
            }
            for (const auto& ap: h_d_n.edges()){
                const auto& ae = e.duration;
                auto a_p_prime = compose(ap, ae);
                double eat_prior = rtasipp::h_dynamic[e.source].earliest_arrival_time();
                rtasipp::add_h_dyn(e.source, a_p_prime);
                double eat = rtasipp::h_dynamic[e.source].earliest_arrival_time();
                if(eat < eat_prior){
                    if (dijkstraOpen.handles.contains(e.source)){
                        auto handle = dijkstraOpen.handles[e.source];
                        dijkstraOpen.decrease_key(handle, eat, e.source);
                    }
                    else{
                        dijkstraOpen.emplace(eat, e.source);
                    }
                }
                
            }
        }
    }
}

std::vector<const SIPPState<Location> *> plrtosipp::search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, long budget, double start_time){
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
        clock_gettime(CLOCK_MONOTONIC, &ts1);
        plrtosipphonly::lsslrtsipp(g, open_list, dest, m);
        plrtolearn(g, open_list, dest, m);
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