#pragma once
#include <bits/types/time_t.h>
#include <boost/functional/hash.hpp>
#include <set>
#include <unordered_map>

#include "constants.hpp"
#include "map.hpp"
#include "structs.hpp"

template <typename Configuration_t>
struct SIPPState{
    Configuration_t configuration;
    atf::interval_t safe_interval;

    SIPPState() = default;
    SIPPState(const Configuration_t& c, const atf::interval_t& si):configuration(c),safe_interval(si){}

    constexpr bool operator ==(const SIPPState& s) const{
        return s.configuration == configuration && s.safe_interval == safe_interval;
    }

    friend std::size_t hash_value(const SIPPState& s){
        std::size_t seed = 0;
        boost::hash_combine(seed, s.configuration.x());
        boost::hash_combine(seed, s.configuration.y());
        boost::hash_combine(seed, s.safe_interval.lower());
        boost::hash_combine(seed, s.safe_interval.upper());
        return seed;
    }
    
    inline friend std::ostream& operator<< (std::ostream& stream, const SIPPState& s){
        stream << s.configuration << " " << s.safe_interval.lower() << " " << s.safe_interval.upper();
        return stream;
    }
};

namespace std {
    template<typename Configuration_t>
    struct hash<SIPPState<Configuration_t>> {
        inline std::size_t operator()(const SIPPState<Configuration_t>& s) const {
           return hash_value(s);
        }
    };
}

template <typename Configuration_t>
struct SIPPEdge{
    SIPPState<Configuration_t> * source;
    SIPPState<Configuration_t> * destination;
    atf::time_t duration;
    atf::interval_t safe_interval;


    SIPPEdge() = default;
    SIPPEdge(SIPPState<Configuration_t> * src, SIPPState<Configuration_t>* dst, atf::time_t dur, atf::interval_t interval):source(src),destination(dst),duration(dur),safe_interval(interval){}

    constexpr bool operator ==(const SIPPEdge& s) const{
        return *s.source == *source && *s.destination == *destination && safe_interval == s.safe_interval;
    }

    friend std::size_t hash_value(const SIPPEdge& s){
        std::size_t seed = 0;
        boost::hash_combine(seed, *s.source);
        boost::hash_combine(seed, *s.destination);
        boost::hash_combine(seed, s.safe_interval.lower());
        boost::hash_combine(seed, s.safe_interval.upper());
        return seed;
    }

    inline friend std::ostream& operator<< (std::ostream& stream, const SIPPEdge& s){
        stream << *s.source << " -> "  << *s.destination << " [" << s.safe_interval.lower() << "," << s.safe_interval.upper() << "): " << s.duration;
        return stream;
    }
};

template <typename Configuration_t>
struct SippGraph{
    std::vector<SIPPState<Configuration_t>> vertices;
    std::unordered_map<const SIPPState<Configuration_t> *, std::vector<SIPPEdge<Configuration_t>>> successors;
    std::unordered_map<const SIPPState<Configuration_t> *, std::vector<SIPPEdge<Configuration_t>>> predecessors;

    SippGraph() = default;

    inline friend std::ostream& operator<< (std::ostream& stream, const SippGraph& g){
        for(const auto& s: g.vertices){
            stream << s << "\n";
            for (const auto& succ: g.successors.at(&s)){
                stream << "\t" << succ << "\n";
            }
        }
        return stream;
    }
};