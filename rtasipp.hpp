#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include "atsippgraph.hpp"
#include "augmentedsipp.hpp"
#include "sippgraph.hpp"

namespace rtasipp{
    using CATF = CompoundATF<std::nullptr_t>;
    extern std::unordered_map<Location, double> h_static;
    extern std::unordered_map<const SIPPState<Location> *, CATF> h_dynamic;

    inline double get_h_s(const Location& cur, const Location& dest){
        if(h_static.find(cur) == h_static.end()){
            h_static[cur] = eightWayDistance(cur, dest);
        }
        return h_static[cur];
    }

    inline double get_h_s(const SIPPState<Location>* cur, const Location& dest){
        return get_h_s(cur->configuration, dest);
    }

    inline double get_h(const SIPPState<Location>& cur, double cur_t, const Location& dest){
        double h_s = get_h_s(&cur, dest);
        if (h_dynamic.find(&cur) == h_dynamic.end()){
            return h_s;
        }   
        const auto & catf = h_dynamic[&cur];
        return catf.arrival_time(cur_t) - cur_t;
    }

    inline void set_h_s(const Location& loc, double x){
        h_static[loc] = x;
    }

    inline void add_h_dyn(const SIPPState<Location> * cur, EdgeATF patf){
        if(h_dynamic.find(cur) == h_dynamic.end()){
            h_dynamic[cur] = CATF();
        }
        h_dynamic[cur].insert(patf, nullptr);
    }

    inline void dump_h_s(std::unordered_map<Location, double> h_static){
        std::cerr << "h_s\n";
        for (auto x: h_static){
            std::cerr << x.first << ": " << x.second << "\n";
        }
        std::cerr << "\n";
    }

   std::vector<const SIPPState<Location> *> search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, double start_time = 0.0, long expansion_budget = -1);
}

