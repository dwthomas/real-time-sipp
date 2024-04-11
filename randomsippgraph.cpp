#include "randomsippgraph.hpp"
#include "constants.hpp"
#include "structs.hpp"
#include "sippgraph.hpp"
#include <unordered_map>
#include <vector>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/discrete_distribution.hpp>

void generate_safe_intervals(std::vector<atf::interval_t>& retval, double until, double occupancy, double min_safe_duration, double max_safe_duration, double avg_unsafe_duration, boost::random::mt11213b& generator){
    if(occupancy == 0.0){
        retval.emplace_back(0.0, until);
        return;
    }

    boost::random::discrete_distribution<> start_safe_flipper({occupancy, 1-occupancy});
    boost::random::uniform_real_distribution<> safe_duration_rnd(min_safe_duration, max_safe_duration);
    boost::random::uniform_real_distribution<> unsafe_duration_rnd(0, 2*avg_unsafe_duration);

    bool issafe = start_safe_flipper(generator);
    double t = 0;
    while(t < until){
        if(issafe){
            double duration = safe_duration_rnd(generator);
            retval.emplace_back(t, t+duration);
            t += duration;
            issafe = false;
        }
        else{
            double duration = unsafe_duration_rnd(generator);
            t += duration;
            issafe = true;
        }
    }
}

SippGraph<Location> make_random_sipp_graph(const Map& map, double until,  double occupancy, double min_duration, double max_duration, const Location& start_location, const Location& goal_location, std::size_t seed){
    if (occupancy < 0 || min_duration < 0 || max_duration < min_duration || until < 0){
        std::cerr << "Invalid random graph requested all rates must be non-negative, occ=" <<occupancy << " min_d=" << min_duration << " max_d=" << max_duration << "\n";
        exit(-1); 
    }

    double avg_safe_duration = 0.5*(min_duration + max_duration);
    double avg_unsafe_duration = occupancy * avg_safe_duration;
    boost::random::mt11213b generator(seed);
    std::vector<std::vector<atf::interval_t>> states;
    states.resize(map.size);
    for(uint y=0; y < map.height; y++){
        for (uint x =0; x < map.width; x++){
            Location l(x, y);
            if(map.isSafe(x, y) && (l == start_location || l == goal_location)){
                states[map.getIndex(l)].emplace_back(0, atf::infty());
            }
            else if(map.isSafe(x, y)){
                generate_safe_intervals(states[map.getIndex(l)], until,  occupancy, min_duration, max_duration, avg_unsafe_duration, generator);
            }   
            else{
                states[map.getIndex(l)].clear();
            }
        }
    }

    SippGraph<Location> g;
    std::unordered_map<SIPPState<Location>, long> indexof;
    // do vertices
    for (std::size_t i = 0; i < states.size(); i++){
        auto loc = map.index2Location(i);
        for(auto interval: states[i]){
            g.vertices.emplace_back(loc,interval);
            indexof[g.vertices.back()] = g.vertices.size()-1;
        }
    }
    // do edges
    SIPPState<Location> vd;
    for (std::size_t i = 0; i < g.vertices.size(); i++){
        SIPPState<Location> * v = &g.vertices[i];
        Location l = v->configuration;
        if(!g.successors.contains(v)){
           g.successors[v].clear(); 
        }
        if(!g.predecessors.contains(v)){
           g.predecessors[v].clear(); 
        }
        for(int dx = -1; dx <= 1; dx++){
            for (int dy = -1; dy <= 1 ; dy++){
                if(dx == 0 && dy == 0){
                    continue;
                }
                else if (dx != 0  && dy != 0) {
                    continue;
                }
                Location sloc(l.x() + dx, l.y() + dy);
                double dist = eightWayDistance(l, sloc);
                atf::interval_t shift_source(v->safe_interval.lower() + dist, v->safe_interval.upper()+dist);
                if(!map.inBounds(sloc.x(), sloc.y())){
                    continue;
                }
                for(const auto& di: states[map.getIndex(sloc)]){
                    if(boost::icl::intersects(shift_source, di)){
                        // valid edge exists
                        vd.configuration = sloc;
                        vd.safe_interval =  di;
                        auto vdi = indexof[vd];
                        g.successors[v].emplace_back(&g.vertices[i], &g.vertices[vdi], dist, atf::interval_t(0, atf::infty()));
                        g.predecessors[&g.vertices[vdi]].emplace_back(&g.vertices[i], &g.vertices[vdi], dist, atf::interval_t(0, atf::infty()));
                    }
                }
            }
        }
    }
    return g;
}