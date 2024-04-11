#include "atsippgraph.hpp"
#include "constants.hpp"
#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

struct inATF{
    long source;
    long dest;
    EdgeATF eATF;
    inATF(long s, long d, EdgeATF e):source(s),dest(d),eATF(e){}
};

void read_ATF(std::istream& i, std::vector<inATF>& res){
    long x, y;
    std::string s;
    if(!(i >> x)){return;}
    i >> y;
    atf::time_t zeta, alpha, beta, delta;
    i >> s;
    zeta = stod(s);
    i >> s;
    alpha = stod(s);
    i >> s;
    beta = stod(s);
    i >> s;
    delta = stod(s);
    EdgeATF edge(zeta, alpha, beta, delta);
    res.emplace_back(x, y, edge); 
}

Atsippgraph read_graph(std::string filename){
    std::ifstream file(filename, std::ios_base::in | std::ios_base::binary);
    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    inbuf.push(boost::iostreams::gzip_decompressor());
    inbuf.push(file);
    std::istream instream(&inbuf);
 
    std::vector<inATF> res;
    Atsippgraph g;
    long n_nodes;
    std::string s;
    instream >> s >> s >> n_nodes;
    g.nodes.reserve(n_nodes);
    g.node_array.reserve(n_nodes);
    for (long i = 0; i < n_nodes; i++){
        gIndex_t x, y;
        double st, en;
        instream >> x;
        instream >> y;
        instream >> s;
        st = stod(s);
        instream >> s;
        en = stod(s);
        State state(x, y, st, en);
        g.node_array.emplace_back(state);
        g.nodes.emplace(state, &g.node_array.back());
    }
    std::cerr << "nodes read\n";

    while(!instream.eof()){
        read_ATF(instream, res);
    }
    file.close();
    // make GraphNodes
    g.edges.reserve(2*res.size());
    for (const auto & entry: res){ 
        g.edges.emplace_back(entry.eATF);
        g.edges.back().source = &g.node_array[entry.source];
        g.edges.back().destination = &g.node_array[entry.dest];
        g.node_array[entry.source].successors.emplace_hint(g.node_array[entry.source].successors.end(), &g.edges.back());
        g.node_array[entry.dest].predecessors.emplace_hint(g.node_array[entry.dest].predecessors.end(), &g.edges.back());
   }
    for(std::size_t i=0; i < g.node_array.size(); i++){
        g.node_array[i].clean();
    }
    return g;
}

AtsippGraphNode *  find_earliest(Atsippgraph& g, Location loc, double start_time){
    AtsippGraphNode * cur = nullptr;
    for (auto& node: g.nodes){
        if (loc == node.first.loc && contains(node.first.interval, start_time) && (cur == nullptr || begin(cur->state.interval) > begin(node.first.interval))){
            cur = node.second;
        }
    }
    if(cur == nullptr){
        std::cerr << "Error: unable to find safe starting state: tried to find ";
        std::cerr << loc << " at time t=" << start_time << "\n";
        exit(-1);
    }
    return cur;
}

std::size_t std::hash<AtsippGraphNode>::operator()(const AtsippGraphNode& e) const{
    std::size_t seed = 0;
    auto s = e.state;
    boost::hash_combine(seed, s.loc.pack());
    boost::hash_combine(seed, s.interval.lower());
    boost::hash_combine(seed, s.interval.upper());
    return seed;
}

std::size_t std::hash<GraphEdge>::operator()(const GraphEdge& e) const{
    std::size_t seed = 0;
    auto s = e.source->state;
    boost::hash_combine(seed, s.loc.pack());
    boost::hash_combine(seed, s.interval.lower());
    boost::hash_combine(seed, s.interval.upper());
    s = e.destination->state;
    boost::hash_combine(seed, s.loc.pack());
    boost::hash_combine(seed, s.interval.lower());
    boost::hash_combine(seed, s.interval.upper());
    return seed;
}

void clean_edges(boost::container::flat_set<GraphEdge *> p){
    std::unordered_set<GraphEdge> edges;
    auto it = p.begin();
    while(it != p.end()){
        auto x = **it;
        if (edges.find(x) != edges.end()){
            it = p.erase(it);
        }
        else{
            edges.emplace(x);
            it++;
        }
    }
}

