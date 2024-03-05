//
// Created by gj on 24. 2. 29.
//

#ifndef RMD_DRIVER_UTILITIES_HPP
#define RMD_DRIVER_UTILITIES_HPP
#pragma once


#include <chrono>
#include <ostream>
#include <ratio>

#include <linux/can.h>
#include <sys/time.h>


namespace rmd_driver{
    template <class Rep, class Period>
    [[nodiscard]]
    constexpr struct ::timeval toTimeval(std::chrono::duration<Rep, Period> const& duration) noexcept {
    auto const usec {std::chrono::duration_cast<std::chrono::duration<Rep, std::micro>>(duration)};
    struct ::timeval tv {};
    tv.tv_sec = static_cast<::time_t>(usec.count()/std::micro::den);
    tv.tv_usec = static_cast<long int>(usec.count())%std::micro::den;
    return tv;
    }
}

#endif //RMD_DRIVER_UTILITIES_HPP
