#pragma once
#include <boost/container/flat_set.hpp>
#include <vector>
#include <set>
#include <limits>
#include <format>

#include "constants.hpp"
#include "segment.hpp"
#include <iostream>

#include <boost/icl/split_interval_map.hpp>
#include <boost/icl/interval.hpp>
#include <cassert>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/intersections.h>


typedef CGAL::Cartesian<double> K;
typedef K::Point_2 Point_2;
typedef K::Segment_2 Segment_2;

struct EdgeATF;

using interval_t = boost::icl::continuous_interval<double>;

struct EdgeATF{
    atf::time_t zeta;
    atf::time_t alpha;
    atf::time_t beta;
    atf::time_t delta;
    std::vector<EdgeATF*> successors;
    EdgeATF() = default;
    EdgeATF(atf::time_t _zeta, atf::time_t _alpha, atf::time_t _beta, atf::time_t _delta):zeta(_zeta),alpha(_alpha),beta(_beta),delta(_delta){}

    inline atf::time_t earliest_arrival_time() const{
        return alpha + delta;
    }
    inline interval_t alphabeta() const{
        interval_t retval(alpha, beta);
        return retval;
    }
    inline interval_t zetaalpha() const{
        interval_t retval(zeta, alpha);
        return retval;
    }
    inline interval_t zetabeta() const{
        interval_t retval(zeta, beta);
        return retval;
    }

    inline atf::time_t arrival_time(atf::time_t t) const{
        if(t < zeta || beta <= t){
            return std::numeric_limits<atf::time_t>::infinity();
        }
        if(t < std::min(alpha, beta)){
            return earliest_arrival_time();
        }
        return t + delta;
    }

    inline atf::time_t inclusive_arrival_time(atf::time_t t) const{
        if(t < zeta || beta < t){
            return std::numeric_limits<atf::time_t>::infinity();
        }
        if(t <= std::min(alpha, beta)){
            return earliest_arrival_time();
        }
        return t + delta;
    }

    inline atf::time_t supremum_arrival_time() const{
        return std::numeric_limits<double>::infinity();
        //return beta + delta;
    }

    inline bool operator<(const EdgeATF& rhs) const{
        return earliest_arrival_time() < rhs.earliest_arrival_time();
    }

    inline bool operator==(const EdgeATF& rhs) const{
        return zeta == rhs.zeta && alpha == rhs.alpha && beta == rhs.beta && delta == rhs.delta;
    }
    
    inline friend std::ostream& operator<< (std::ostream& stream, const EdgeATF& eatf){
        stream << "<" << eatf.zeta << "," << eatf.alpha << "," << eatf.beta << "," << eatf.delta << ">";
        return stream;
    }

    inline segments_small_container segments() const{
        segments_small_container res;
        double periapsis = arrival_time(alpha);
        double apoapsis = inclusive_arrival_time(beta);
        if(alpha > zeta){
            res.emplace_back(zeta, alpha, periapsis, periapsis, -1);
        }
        if(beta > alpha){
            res.emplace_back(alpha, beta, periapsis, apoapsis, -1);
        }
        return res;
    }
};

namespace std {
    template<>
    struct hash<EdgeATF> {
        inline std::size_t operator()(const EdgeATF& eatf) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, eatf.zeta);
            boost::hash_combine(seed, eatf.alpha);
            boost::hash_combine(seed, eatf.beta);
            boost::hash_combine(seed, eatf.delta);
            return seed;
        }
    };
}

inline EdgeATF shiftIdentity(double x){
    return EdgeATF(0, 0, std::numeric_limits<double>::infinity(), x);
}

using EdgeATFList = boost::container::flat_set<EdgeATF>;

inline EdgeATF compose(const EdgeATF& e_prime, const EdgeATF& e){
    double zeta = e.zeta;
    double alpha = std::max(e.alpha, e_prime.alpha - e.delta);
    double beta = std::min(e.beta, e_prime.beta - e.delta);
    double delta = e.delta + e_prime.delta;
    return EdgeATF(zeta, alpha, beta, delta);
}

template <typename payload_T>
struct EdgeATFholster{
    int size;
    EdgeATF encumbent;
    EdgeATF newcomer;
    payload_T enc_payload;
    payload_T new_payload;
    
    EdgeATFholster(){
        size = 0;
        encumbent = EdgeATF();
        newcomer = EdgeATF();
        enc_payload = payload_T();
        new_payload = payload_T();
    }

    EdgeATFholster(const EdgeATF& e, payload_T p){
        size = 1;
        encumbent = e;
        enc_payload = p;
        newcomer = EdgeATF();
        new_payload = payload_T();
    }

    bool full() const{
        return size == 2;
    }

    std::pair<std::pair<interval_t, EdgeATFholster>, std::pair<interval_t, EdgeATFholster>> intersection(const interval_t & interval) const{
        assert(full());
        double enc_y0, enc_y1, new_y0, new_y1;
        enc_y0 = encumbent.arrival_time(interval.lower());
        enc_y1 = encumbent.inclusive_arrival_time(interval.upper());
        new_y0 = newcomer.arrival_time(interval.lower());
        new_y1 = newcomer.inclusive_arrival_time(interval.upper());
        Segment_2 a(Point_2(interval.lower(), enc_y0), Point_2(interval.upper(), enc_y1));
        Segment_2 b(Point_2(interval.lower(), new_y0), Point_2(interval.upper(), new_y1));
        auto x = CGAL::intersection(a, b);
        const Point_2* p = boost::get<Point_2 >(&*x);
        if(p == nullptr){
            std::cerr << interval.lower() << " " << interval.upper() << "\n";  
            std::cerr << encumbent << "\n" << newcomer << "\n";
            std::cerr <<enc_y0 << " " <<  enc_y1 << " " <<  new_y0 << " " << new_y1 << "\n";
        }
        double inter = CGAL::to_double(p->x());
        interval_t left(interval.lower(), inter), right(inter, interval.upper());
        EdgeATFholster lh, rh;
        if(enc_y0 < new_y0){
            lh.size = 1;
            lh.encumbent = encumbent;
            lh.enc_payload = enc_payload;
            rh.size = 1;
            rh.encumbent = newcomer;
            rh.enc_payload = new_payload;
        }
        else{
            lh.size = 1;
            lh.encumbent = newcomer;
            lh.enc_payload = new_payload;
            rh.size = 1;
            rh.encumbent = encumbent;
            rh.enc_payload = enc_payload;
        }
        return std::make_pair(
            std::make_pair(left, lh),
            std::make_pair(right, rh)
        );
    }

    EdgeATFholster dominant(const interval_t& interval) const{
        assert(full());
        double enc_y0, enc_y1, new_y0, new_y1;
        enc_y0 = encumbent.arrival_time(interval.lower());
        enc_y1 = encumbent.inclusive_arrival_time(interval.upper());
        new_y0 = newcomer.arrival_time(interval.lower());
        new_y1 = newcomer.inclusive_arrival_time(interval.upper());
        //std::cerr << enc_y0 << " " << enc_y1 << " " << new_y0 << " " << new_y1 << "\n";
        if (enc_y0 <= new_y0 && enc_y1 <= new_y1){
            return EdgeATFholster(encumbent, enc_payload);
        }
        else if(enc_y0 >= new_y0 && enc_y1 >= new_y1){
            return EdgeATFholster(newcomer, new_payload);
        }   
        return EdgeATFholster(*this);     
    }

    EdgeATFholster& operator+=(const EdgeATFholster& right){
        
        assert(size == 1);
        size = 2;
        newcomer = right.encumbent;
        new_payload = right.enc_payload;
        return *this;
    }

   
};

template <typename payload_T>
bool operator==(const EdgeATFholster<payload_T>& left, const EdgeATFholster<payload_T>& right){ 
    return left.encumbent == right.encumbent && left.newcomer == right.newcomer;
}

template <typename payload_T>
using interval_map_t = boost::icl::split_interval_map<double, EdgeATFholster<payload_T>>; 

template <typename payload_T>
class CompoundATF{
    private:
        interval_map_t<payload_T> segments;

      void fix_redundant(interval_t interval){
            auto it = segments.find(interval.lower());
            interval_map_t<payload_T> new_segments;
            std::vector<std::pair<const interval_t, EdgeATFholster<payload_T>>> to_erase;
            while(it != segments.end() && it->first.lower() <= interval.upper()){
                if(it->second.full()){
                    auto rv = it->second.dominant(it->first);
                    if(rv.full()){
                        auto inter = it->second.intersection(it->first);
                        new_segments.insert(inter.first);
                        new_segments.insert(inter.second);
                        auto x = *it;
                    }
                    else{
                        new_segments.insert(std::make_pair(it->first, rv));
                    }
                    to_erase.emplace_back(*it);
                }
                it++;
            }
            for(auto x: to_erase){
                segments.erase(x);
            }
            for(auto x: new_segments){
                segments.insert(x);
            }            
        }

        void insert_segment(interval_t interval, EdgeATF e, payload_T p){
            EdgeATFholster es(e, p);
            segments += std::make_pair(interval, es);
            fix_redundant(interval);
        } 
    public:
        CompoundATF() = default;
        
        inline void insert(EdgeATF e, payload_T p){
            if(e.beta <= e.alpha){
                insert_segment(e.zetabeta(), e, p);
            }
            else{
                insert_segment(e.zetaalpha(), e, p);
                insert_segment(e.alphabeta(), e, p);
            }
         
        }

        inline std::pair<interval_t, EdgeATF> at(double t) const{
            auto acc = segments.find(t);
            return std::make_pair(acc->first, acc->second.encumbent);
        }        

        inline payload_T payload_at(double t) const{
            auto acc = segments.find(t);
            return acc->second.enc_payload;
        }

        inline double arrival_time(double t) const{
            auto acc = segments.find(t);
            if(acc == segments.end()){
                return std::numeric_limits<double>::infinity();
            }
            auto x = at(t);
            return x.second.arrival_time(t);
        }

        inline double earliest_arrival_time() const{
            auto acc = segments.begin();
            if (acc == segments.end()){
                return std::numeric_limits<double>::infinity();
            }
            return acc->second.encumbent.earliest_arrival_time();
        }

        inline std::unordered_set<EdgeATF> edges() const{
            std::unordered_set<EdgeATF> retval;
            for (auto seg: segments){
                retval.emplace(seg.second.encumbent);
            }
            return retval;
        }

        inline void dump(std::ostream& stream) const{
            for (auto seg: segments){
                stream << seg.first;
                stream << " " << seg.second.encumbent;
                if(seg.second.full()){
                    stream << " " << seg.second.newcomer;
                } 
                stream << std::endl;
            }
        }

        inline friend std::ostream& operator<<(std::ostream& stream, const CompoundATF& eatf){
            eatf.dump(stream);
            return stream;
        }   

};