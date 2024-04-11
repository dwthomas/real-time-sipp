#pragma once
#include "atf.hpp"
#include "sippgraph.hpp"
#include "structs.hpp"
#include <boost/multiprecision/detail/number_base.hpp>

template <typename Configuration_t>
struct AtSIPPEdge{
    SIPPState<Configuration_t> * source;
    SIPPState<Configuration_t> * destination;
    EdgeATF duration;


    AtSIPPEdge() = default;
    AtSIPPEdge(SIPPState<Configuration_t> * src, SIPPState<Configuration_t>* dst, EdgeATF dur):source(src),destination(dst),duration(dur){}

    constexpr bool operator ==(const AtSIPPEdge& s) const{
        return *s.source == *source && *s.destination == *destination;
    }

    friend std::size_t hash_value(const AtSIPPEdge& s){
        std::size_t seed = 0;
        boost::hash_combine(seed, *s.source);
        boost::hash_combine(seed, *s.destination);
        return seed;
    }

    inline friend std::ostream& operator<< (std::ostream& stream, const AtSIPPEdge& s){
        stream << *s.source << " -> "  << *s.destination << ": " << s.duration;
        return stream;
    }
};

template <typename Configuration_t>
AtSIPPEdge<Configuration_t> compile(const SIPPEdge<Configuration_t>& sipp_edge){
    const SIPPState<Configuration_t>& u = *sipp_edge.source;
    const SIPPState<Configuration_t>& v = *sipp_edge.destination;
    double zeta = u.safe_interval.lower();
    double alpha = std::max(
        sipp_edge.safe_interval.lower(),
        std::max(
            u.safe_interval.lower(),
            v.safe_interval.lower() - sipp_edge.duration
        )
    );
    double beta = std::min(
        sipp_edge.safe_interval.upper(),
        std::min(
            u.safe_interval.upper(),
            v.safe_interval.upper() - sipp_edge.duration
        )
    );
    double delta = sipp_edge.duration;
    EdgeATF e(zeta, alpha, beta, delta);
    return AtSIPPEdge<Configuration_t>(sipp_edge.source, sipp_edge.destination, e);
}

template <typename Configuration_t>
struct AtSippGraph{
    const SippGraph<Configuration_t> * sipp_graph;
    std::unordered_map<const SIPPState<Configuration_t> *, std::vector<AtSIPPEdge<Configuration_t>>> successors;
    std::unordered_map<const SIPPState<Configuration_t> *, std::vector<AtSIPPEdge<Configuration_t>>> predecessors;

    AtSippGraph() = default;

    AtSippGraph(const SippGraph<Configuration_t> * g):sipp_graph(g){

        for(const auto& s: sipp_graph->vertices){
            const auto& succ = sipp_graph->successors.at(&s);
            successors[&s].reserve(succ.size());
            for (const auto& successor: succ){
                successors[&s].emplace_back(compile(successor));
            }
            const auto& pred = sipp_graph->predecessors.at(&s);
            predecessors[&s].reserve(pred.size());
            for (const auto& predecessor: pred){
                predecessors[&s].emplace_back(compile(predecessor));
            }
        }
    }

    inline friend std::ostream& operator<< (std::ostream& stream, const AtSippGraph& g){
        stream << "g size: "  << g.successors.size() << "\n";
        for(const auto& x: g.successors){
                stream << x.first << " " <<  *x.first <<  "\n"; 
        }   
        for(std::size_t i = 0; i < g.sipp_graph->vertices.size(); i++){
            const auto * s = &g.sipp_graph->vertices[i];            
            for (const auto& succ: g.successors.at(s)){
                stream << "\t" << succ << "\n";
            }
        }
        return stream;
    }
};

template <typename Configuration_t>
const SIPPState<Configuration_t> *  find_earliest(const SippGraph<Configuration_t>& g, Configuration_t loc, double start_time){
    SIPPState<Configuration_t> * cur = nullptr;
    for (std::size_t i = 0; i < g.vertices.size(); i++){
        const auto& v = g.vertices.at(i);
        if (loc == v.configuration && contains(v.safe_interval, start_time)){
            return &v;
        }
    }
    if(cur == nullptr){
        std::cerr << "Error: unable to find safe starting state: tried to find ";
        std::cerr << loc << " at time t=" << start_time << "\n";
        exit(-1);
    }
    return cur;
}