#pragma once
#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include "atf.hpp"
#include "augmentedsipp.hpp"
#include "sippgraph.hpp"

namespace plrtosipp{
    struct DijkstraNode{
        EdgeATF g;
        const SIPPState<Location> * node;
        DijkstraNode() = default;
        DijkstraNode(EdgeATF e, const SIPPState<Location> * _node):g(e),node(_node){}

        inline friend bool operator>(const DijkstraNode& a, const DijkstraNode& b){
            return a.g.earliest_arrival_time() > b.g.earliest_arrival_time();
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const DijkstraNode& n){
            stream << *n.node << " g:" << n.g;
            return stream;
        }
    };

    struct DijkstraNodeComp{
        bool operator()(const DijkstraNode * a, const DijkstraNode * b){
            return *a > *b;
        }
    };
    using DijkstraQueue = boost::heap::d_ary_heap<DijkstraNode, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<std::greater<DijkstraNode>>>;
    typedef typename DijkstraQueue::handle_type Dijkstra_handle_t;


    struct DijkstraOpen{
        DijkstraQueue queue;
        std::unordered_map<const SIPPState<Location> *, Dijkstra_handle_t> handles;

        inline void emplace(EdgeATF g, const SIPPState<Location> * n){
            handles[n] = queue.push(DijkstraNode(g,  n));
        }

        inline bool empty() const{
            return queue.empty();
        }

        inline DijkstraNode top() const{
            return queue.top();
        }

        inline void pop(){
            queue.pop();
        }

        inline void decrease_key(Dijkstra_handle_t handle , EdgeATF g, const SIPPState<Location> * n){
            queue.update(handle, DijkstraNode(g, n));
        }
    };

    struct LSSNode{
        double g;
        const SIPPState<Location> * node;
        LSSNode() = default;
        LSSNode(double _g,  const SIPPState<Location> * _node):g(_g),node(_node){}

        inline friend bool operator>(const LSSNode& a, const LSSNode& b){
            return a.g > b.g;
        }

        inline friend std::ostream& operator<< (std::ostream& stream, const LSSNode& n){
            stream << *n.node << " g:" << n.g;
            return stream;
        }
    };

    struct LSSNodeComp{
        bool operator()(const LSSNode * a, const LSSNode * b){
            return *a > *b;
        }
    };
    using LSSQueue = boost::heap::d_ary_heap<LSSNode, boost::heap::arity<4>, boost::heap::mutable_<true>, boost::heap::compare<std::greater<LSSNode>>>;
    typedef typename LSSQueue::handle_type LSS_handle_t;

    struct LSSOpen{
        LSSQueue queue;
        std::unordered_map<const SIPPState<Location> *, LSS_handle_t> handles;

        inline void emplace(double g, const SIPPState<Location> * n){
            handles[n] = queue.push(LSSNode(g, n));
        }

        inline bool empty() const{
            return queue.empty();
        }

        inline LSSNode top() const{
            return queue.top();
        }

        inline void pop(){
            queue.pop();
        }

        inline void decrease_key(LSS_handle_t handle , double g, const SIPPState<Location> * n){
            queue.increase(handle, LSSNode(g, n));
        }
    };

    void plrtolearn(const AtSippGraph<Location> & g, const asipp::Open& open_list, const Location& dest, MetaData& m);
    std::vector<const SIPPState<Location> *> search(const AtSippGraph<Location> & g, const SIPPState<Location> * source, const Location& dest, MetaData & m, long budget, double start_time = 0.0);
}

