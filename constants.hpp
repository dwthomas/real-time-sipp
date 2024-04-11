#pragma once
#include <cstdint>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <boost/icl/interval.hpp>

// BOOST_ENABLE_ASSERT_DEBUG_HANDLER is defined for the whole project

using gIndex_t = uint16_t;

namespace atf{
    using time_t = double;
    using interval_t = boost::icl::right_open_interval<atf::time_t>;
    constexpr time_t infty(){
        return std::numeric_limits<time_t>::infinity();
    }
}

constexpr double epsilon(){
    return 0.0001;
}

constexpr double sqrt2(){
    return boost::math::double_constants::root_two;
}

