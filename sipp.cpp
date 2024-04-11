#include "sipp.hpp"
#include "sippgraph.hpp"
#include "structs.hpp"
#include <algorithm>
#include <time.h>

using namespace sipp;

bool isGoal(const Node& n, const Location& goal_loc){
    return n.state->configuration == goal_loc;
}

void expand(const SippGraph<Location>& g, const Node& cur, Open& open_list, const Location& goal_loc, MetaData & m){
    m.expanded++;
    auto successors = g.successors.at(cur.state);
    for(const auto & successor : successors){
        if(open_list.expanded.contains(successor.destination)){
            continue;
        }
        double earliest_arrival_time = std::max(
            cur.g + successor.duration, // no wait
            std::max(
                successor.safe_interval.lower() + successor.duration,
                successor.destination->safe_interval.lower()
            )
        );
        if(!contains(cur.state->safe_interval, earliest_arrival_time - successor.duration) || 
           !contains(successor.safe_interval, earliest_arrival_time - successor.duration) || 
           !contains(successor.destination->safe_interval, earliest_arrival_time)){
            continue;
        }
        if (open_list.handles.contains(successor.destination)){
            auto handle = open_list.handles[successor.destination];
            if(earliest_arrival_time < (*handle).g){
                m.decreased++;
                double h = eightWayDistance(successor.destination->configuration, goal_loc);
                open_list.decrease_key(handle, earliest_arrival_time, h, successor.destination, successor.source);
            }
        }
        else{
            m.generated++;
            double h = eightWayDistance(successor.destination->configuration, goal_loc);
            open_list.emplace(earliest_arrival_time, h, successor.destination, successor.source);
        }
    }
}

void dump_open(const Open& open_list){
    Queue::ordered_iterator cur = open_list.queue.ordered_begin();
    Queue::ordered_iterator end = open_list.queue.ordered_end();
    std::cerr << "Open:";
    while(cur != end){
        std::cerr << "\t" << *cur << "\n";
        cur = std::next(cur);
    }
}

std::vector<const SIPPState<Location> *> backup(const Node& n, Open& open_list){
    std::vector<const SIPPState<Location> *> res;
    const SIPPState<Location> * cur = n.state;
    while(cur != nullptr){
        res.push_back(cur);
        cur = open_list.parent[cur];
    }
    std::reverse(res.begin(), res.end());
    std::cout << "Arrival time: " << n.f << "\n";
    return res;
}

std::vector<const SIPPState<Location> *> sipp::search(const SippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData& m, double start_time){
    Open open_list;
    m.init();
    struct timespec ts1, ts2;
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    open_list.emplace(start_time, eightWayDistance(dest, source->configuration), source, nullptr);
    while(!open_list.empty()){
        Node cur = open_list.top();
        if(isGoal(cur, dest)){
            clock_gettime(CLOCK_MONOTONIC, &ts2);
            m.search_time = 1000.0 * ts2.tv_sec + 1e-6 * ts2.tv_nsec - (1000.0 * ts1.tv_sec + 1e-6 * ts1.tv_nsec);
            return backup(cur, open_list);
        }
        open_list.pop();
        expand(g, cur, open_list, dest, m);
    }
    std::cerr << "Failed to find path\n";
    exit(-1);
}