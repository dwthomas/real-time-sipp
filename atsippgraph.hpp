#pragma once
#include "structs.hpp"
#include "atf.hpp"
#include <limits>
#include <unordered_map>
#include <boost/container/flat_set.hpp>
#include <boost/unordered/unordered_flat_map.hpp>
#include "sippgraph.hpp"

struct GraphEdge;
struct AtsippGraphNode;

void clean_edges(boost::container::flat_set<GraphEdge *> p);

struct AtsippGraphNode{
    State state;
    boost::container::flat_set<GraphEdge *> predecessors;
    boost::container::flat_set<GraphEdge *> successors;
    AtsippGraphNode() = default;
    AtsippGraphNode(const State& s):state(s){}
    inline friend std::ostream& operator<<(std::ostream& stream, const AtsippGraphNode& gn){
        stream << gn.state << " ns:" << gn.successors.size();
        return stream;
    }
    constexpr friend double operator-(const AtsippGraphNode& lhs, const AtsippGraphNode& rhs){
        return eightWayDistance(lhs.state.loc, rhs.state.loc);
    }

    inline friend bool operator==(const AtsippGraphNode& lhs, const AtsippGraphNode& rhs){
        return lhs.state == rhs.state;
    }

    inline void clean(){
        clean_edges(predecessors);
        clean_edges(successors);
    }
};




struct GraphEdge{
    EdgeATF edge;
    AtsippGraphNode * source;
    AtsippGraphNode * destination;
    GraphEdge(const EdgeATF& e):edge(e){
        source = nullptr;
        destination = nullptr;
    }
    inline friend std::ostream& operator<< (std::ostream& stream, const GraphEdge& ge){
        stream << ge.edge << " " << *ge.source << "->" << *ge.destination;
        return stream;
    }

    inline friend bool operator<(const GraphEdge& lhs, double rhs){
        return lhs.edge.earliest_arrival_time() < rhs;
    }

    inline friend bool operator<(const GraphEdge& lhs, const GraphEdge& rhs){
        return lhs < rhs.edge.earliest_arrival_time();
    }

    inline friend bool operator==(const GraphEdge& lhs, const GraphEdge& rhs){
        return lhs.edge == rhs.edge && *lhs.source == *rhs.source && *lhs.destination == *rhs.destination;
    }
};

namespace std {
    template<>
    struct hash<AtsippGraphNode> {
        std::size_t operator()(const AtsippGraphNode& e) const;
    };
}

namespace std {
    template<>
    struct hash<GraphEdge> {
        std::size_t operator()(const GraphEdge& e) const ;
    };
}

struct Atsippgraph{
    std::vector<GraphEdge> edges;
    std::vector<AtsippGraphNode> node_array;
    boost::unordered::unordered_flat_map<State, AtsippGraphNode *, std::hash<State>> nodes;
    Atsippgraph() = default;
    inline void dump() const{
        for (const auto & n: nodes){
            std::cout << *n.second;
            std::cout << " succ:\n";
            for (const auto& s: n.second->successors){
                std::cout << "\t" << *s  << "\n";
            }
            std::cout << "\n";
        }
        for (const auto& e: edges){
            std::cout << e << "\n";
        }
    }
    inline friend std::ostream& operator<< (std::ostream& stream, const Atsippgraph& g){
        stream << g.edges.size() << " edges, " << g.nodes.size() << " nodes";       
        return stream;
    }
};

Atsippgraph read_graph(std::string filename);
AtsippGraphNode * find_earliest(Atsippgraph& g, Location loc, double start_time);

