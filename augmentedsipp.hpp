#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <functional>
#include <unordered_map>
#include "newatsippgraph.hpp"
#include "sippgraph.hpp"

namespace asipp{
    struct Node;



    double constexpr h_eight_way_helper(const SIPPState<Location>& cur, double cur_t, const Location& dest){
        (void) cur_t;
        return eightWayDistance(cur.configuration, dest);
    }

    struct Node{
        EdgeATF g;
        double f;
        const SIPPState<Location> * state;
        const AtSIPPEdge<Location> * tla;
        Node() = default;
        Node(EdgeATF e, double _h, const SIPPState<Location> * _state, const AtSIPPEdge<Location> * _tla):g(e),f(e.earliest_arrival_time() + _h),state(_state),tla(_tla){}

        Node(double _f, EdgeATF e, const SIPPState<Location> * _state, const AtSIPPEdge<Location> * _tla):g(e),f(_f),state(_state),tla(_tla){}

        inline friend bool operator>(const Node& a, const Node& b){
            if(a.f == b.f){
                return a.g < b.g;
            }
            return a.f > b.f;
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const Node& n){
            stream << *n.state << " g:" << n.g << ", f:" << n.f;
            return stream;
        }
    };

    struct NodeComp{
        bool operator()(const Node * a, const Node * b){
            std::cerr << "comp " << *a << " " << *b << " " << "\n";
            return *a > *b;
        }
    };

    using Queue = boost::heap::d_ary_heap<Node, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<std::greater<Node>>>;
    typedef typename Queue::handle_type handle_t;
    struct Open{
        Queue queue;
        std::unordered_map<const SIPPState<Location> *, const SIPPState<Location> *> parent;
        std::unordered_map<const SIPPState<Location> *, handle_t> handles;
        std::unordered_map<const SIPPState<Location> *, double> expanded;

        inline void emplace(EdgeATF e, double h, const SIPPState<Location> * n, const SIPPState<Location> * p, const AtSIPPEdge<Location> * tla){
            parent[n] = p;
            handles[n] = queue.push(Node(e, h, n, tla));
        }

        inline bool empty() const{
            return queue.empty();
        }

        inline Node top() const{
            return queue.top();
        }

        inline void pop(){
            Node n = top();
            expanded[n.state] = n.g.earliest_arrival_time();
            queue.pop();
        }

        inline void decrease_key(handle_t handle , EdgeATF e, double h, const SIPPState<Location> * n, const SIPPState<Location> * p, const AtSIPPEdge<Location> * tla){
            parent[n] = p;
            queue.update(handle, Node(e, h, n, tla));
        }
    };

    template <typename Node_t>
    bool isGoal(const Node_t& n, const Location& goal_loc){
        return n.state->configuration == goal_loc;
    }


    template <typename Node_t, typename Open_t>
    std::pair<std::vector<const SIPPState<Location> *>, double> backup(const Node_t& n, Open_t& open_list){
        std::vector<const SIPPState<Location> *> res;
        const SIPPState<Location>* cur = n.state;
        while(cur != nullptr){
            res.push_back(cur);
            cur = open_list.parent[cur];
        }
        std::reverse(res.begin(), res.end());
        return std::make_pair(res, n.f);
    }

    template <typename Node_t, typename Open_t>
    inline void expand(const AtSippGraph<Location>& g, const Node_t& cur, Open_t& open_list, const Location& goal_loc, MetaData & m, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
        m.expanded++;
        double zeta = cur.g.zeta;
        for(const AtSIPPEdge<Location>& successor: g.successors.at(cur.state)){
            if(cur.g.earliest_arrival_time() >= successor.duration.beta || cur.g.supremum_arrival_time() <= successor.duration.zeta){
                continue;
            } 
            auto tla = cur.tla;
            if (tla == nullptr){
                tla = &successor;
            }
            double alpha = std::max(cur.g.alpha, successor.duration.alpha - cur.g.delta);
            double beta = std::min(cur.g.beta, successor.duration.beta - cur.g.delta);
            double delta = successor.duration.delta + cur.g.delta;
            EdgeATF arrival_time_function(zeta, alpha, beta, delta);
            if(open_list.expanded.contains(successor.destination)){
                continue;
            }
            else if (open_list.handles.contains(successor.destination)){
                auto handle = open_list.handles[successor.destination];
                if(arrival_time_function.earliest_arrival_time() < (*handle).g.earliest_arrival_time()){
                    m.decreased++;
                    double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
                    open_list.decrease_key(handle ,arrival_time_function, h, successor.destination, successor.source, tla);
                }
            }
            else{
                m.generated++;
                double h = hf(*successor.destination, arrival_time_function.earliest_arrival_time(), goal_loc);
                open_list.emplace(arrival_time_function, h, successor.destination, successor.source, tla);
            }
        }
    }

    template<typename Open_t>
    inline void dump_open(const Open_t& open_list){
        auto cur = open_list.queue.ordered_begin();
        auto end = open_list.queue.ordered_end();
        std::cerr << "Open:\n";
        while(cur != end){
            std::cerr << "\t" << *cur << "\n";
            cur = std::next(cur);
        }
        std::cerr << "Closed:\n";
        for( auto x : open_list.expanded){
            std::cerr <<  "\t" << *x.first << "\n";
        }

    }
    template<typename Open_t>
    inline std::pair<std::vector<const SIPPState<Location> *>, EdgeATF> search_core(const AtSippGraph<Location>& g, Open_t& open_list, const Location& dest, MetaData & m, long expansion_budget = -1, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper){
        long start_expansions = m.expanded;
        while(!open_list.empty()){
            auto cur = open_list.top();
            if(isGoal(cur, dest) || (expansion_budget >= 0 && m.expanded - start_expansions >= expansion_budget)){
                return std::make_pair(backup(cur, open_list).first, cur.g);
            }
            open_list.pop();
            expand(g, cur, open_list, dest, m, hf);
        }
        std::cerr << "Failed to find path\n";
        exit(-1);
    }

    std::pair<std::vector<const SIPPState<Location> *>, EdgeATF> search(const AtSippGraph<Location>& g, const SIPPState<Location> * source, const Location& dest, MetaData & m, double start_time = 0.0, long expansion_budget = -1, double (*hf)(const SIPPState<Location>&, double , const Location& ) = h_eight_way_helper);
}

